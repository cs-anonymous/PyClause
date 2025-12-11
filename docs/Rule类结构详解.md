# Rule 类结构详解

## 一、Rule 基类概览

`Rule` 是一个抽象基类，定义了规则的通用接口和数据结构。所有具体的规则类型（RuleB, RuleC, RuleD, RuleZ, RuleXXc, RuleXXd, RuleM）都继承自这个基类。

---

## 二、成员变量详解

### 1. 公有成员（Public）

#### `bool predictHead`
- **用途**：标识该规则是否可用于预测头实体（head query）
- **默认值**：通常为 `true`
- **说明**：某些特殊规则（如从 AnyBURL 解析的 UXX 规则）可能只支持单向预测

#### `bool predictTail`
- **用途**：标识该规则是否可用于预测尾实体（tail query）
- **默认值**：通常为 `true`
- **说明**：与 `predictHead` 配合，控制规则的预测方向

#### `const char* type`
- **用途**：规则类型标识符
- **可能的值**：
  - `"b"` - RuleB（闭合连接规则）
  - `"c"` - RuleC（带 2 个常量的规则）
  - `"d"` - RuleD（悬挂原子规则）
  - `"z"` - RuleZ（零体规则）
  - `"xxd"` - RuleXXd（自环悬挂规则）
  - `"xxc"` - RuleXXc（自环常量规则）
  - `"m"` - RuleM（多规则组合）
- **说明**：用于运行时类型判断和配置过滤

---

### 2. 受保护成员（Protected）

#### 规则标识与统计

##### `int ID`
- **用途**：规则的唯一标识符
- **默认值**：`-1`（未分配）
- **说明**：在规则加载时由 RuleStorage 分配，用于索引和排序

##### `int length`
- **用途**：规则体（body）中的原子数量
- **默认值**：`0`
- **示例**：
  - `h(X,Y) <= b1(X,A), b2(A,Y)` → `length = 2`
  - `h(X,c) <=` → `length = 0`

##### `long long bodyhash`
- **用途**：规则体的哈希值（保留字段）
- **默认值**：`0`
- **说明**：可用于快速比较规则体是否相同（目前未实现）

#### 预测统计（训练集）

##### `int predicted`
- **用途**：规则在训练集中的**总预测次数**（body groundings）
- **默认值**：`0`
- **说明**：分母，用于计算置信度
- **示例**：规则体被实例化（grounded）了 100 次

##### `int cpredicted`
- **用途**：规则在训练集中的**正确预测次数**（correct predictions）
- **默认值**：`0`
- **说明**：分子，用于计算置信度
- **示例**：100 次预测中有 75 次在训练集中存在

##### `int sampledPredicted`
- **用途**：采样统计的总预测次数
- **默认值**：`0`
- **说明**：当使用采样方式计算置信度时使用

##### `int sampledCpredicted`
- **用途**：采样统计的正确预测次数
- **默认值**：`0`
- **说明**：与 `sampledPredicted` 配合计算采样置信度

#### 置信度计算参数

##### `double confWeight`
- **用途**：置信度的权重系数
- **默认值**：`1.0`
- **特殊情况**：
  - RuleZ（零体规则）：默认 `0.01`
  - RuleD（悬挂规则）：默认 `0.1`
- **公式**：`confidence = confWeight × (cpredicted / (predicted + numUnseen))`

##### `int numUnseen`
- **用途**：拉普拉斯平滑参数
- **默认值**：`5`
- **说明**：防止置信度过度拟合，给未见过的情况留余地
- **公式影响**：增大 `numUnseen` 会降低置信度（更保守）

#### 行为控制

##### `bool trackInMaterialize`
- **用途**：是否在物化（materialize）过程中跟踪统计信息
- **默认值**：`false`
- **说明**：设为 `true` 时，物化过程会更新 `predicted` 和 `cpredicted`

##### `int branchingFactor`
- **用途**：DFS 搜索时的分支限制
- **默认值**：`-1`（无限制）
- **说明**：
  - 当某个中间节点的邻居数超过此值时，停止该分支搜索
  - 用于大型知识图谱（如 Wikidata5M）防止搜索爆炸
  - RuleB 默认：`1000`

#### 内部表示

##### `std::string rulestring`
- **用途**：规则的字符串表示（缓存）
- **默认值**：对象指针地址
- **说明**：通过 `computeRuleString()` 生成并缓存

##### `std::vector<int> relations`
- **用途**：规则中所有关系的 ID 序列
- **结构**：`[head_rel, body_rel_1, body_rel_2, ...]`
- **示例**：
  ```cpp
  // h(X,Y) <= b1(X,A), b2(A,Y)
  relations = [id(h), id(b1), id(b2)]
  ```

##### `std::vector<bool> directions`
- **用途**：每个 body 原子的方向
- **长度**：`relations.size() - 1`（不包括 head）
- **含义**：
  - `true`：正向，`(var_i, var_{i+1})`
  - `false`：反向，`(var_{i+1}, var_i)`
- **示例**：
  ```cpp
  // b1(X,A), b2(Y,A)  -- 第二个原子是反向的
  directions = [true, false]
  ```

##### `int targetRel`
- **用途**：头部关系的 ID（即预测的目标关系）
- **默认值**：`0`
- **说明**：通常等于 `relations[0]`

---

### 3. 私有成员（Private）

基类没有私有成员。各个子类有自己的私有成员，用于存储类型特定的信息（如常量、内部优化的表示等）。

---

## 三、如何构造 Rule

### 3.1 构造流程

规则不是直接通过 `new Rule()` 创建的，而是通过 **RuleFactory** 解析字符串生成：

```
字符串规则 → RuleFactory::parseAnytimeRule() → 具体规则类型对象
```

### 3.2 解析过程（以 RuleB 为例）

#### 输入：规则字符串
```
"100\t75\t0.75\th(X,Y) <= b1(X,A), b2(A,Y)"
```

#### 步骤 1：分割统计信息和规则字符串
```cpp
stats: predicted=100, cpredicted=75, conf=0.75
rule_str: "h(X,Y) <= b1(X,A), b2(A,Y)"
```

#### 步骤 2：解析头部（head）
```cpp
parseAtom("h(X,Y)") 
→ headAtom = {relation: "h", left: "X", right: "Y"}
→ relID = index->getIdOfRelationstring("h")
→ relations = [relID]
```

#### 步骤 3：解析体部（body）
```cpp
split by " <= " → body_str = "b1(X,A), b2(A,Y)"
split by ", " → ["b1(X,A)", "b2(A,Y)"]

// 对每个 body atom：
parseAtom("b1(X,A)")
→ bodyAtom[0] = {relation: "b1", left: "X", right: "A"}
→ relations.push_back(index->getIdOfRelationstring("b1"))

// 判断方向：
// X 是 var[0], A 是 var[1]
// b1(X,A) 是正向 → directions.push_back(true)

parseAtom("b2(A,Y)")
→ bodyAtom[1] = {relation: "b2", left: "A", right: "Y"}
→ relations.push_back(index->getIdOfRelationstring("b2"))

// A 是 var[1], Y 是 var[2] (last var)
// b2(A,Y) 是正向 → directions.push_back(true)
```

#### 步骤 4：创建 RuleB 对象
```cpp
std::unique_ptr<RuleB> ruleb = std::make_unique<RuleB>(relations, directions);

// RuleB 构造函数内部：
this->relations = relations;          // [h_id, b1_id, b2_id]
this->directions = directions;        // [true, true]
this->targetRel = relations.front(); // h_id
this->type = "b";
this->length = directions.size();    // 2
```

#### 步骤 5：设置统计信息和配置
```cpp
ruleb->setStats(cpredicted=75, predicted=100, exact=false);
ruleb->setNumUnseen(5);              // 从配置读取
ruleb->setBranchingFactor(1000);     // 从配置读取
```

#### 步骤 6：返回规则对象
```cpp
return std::move(ruleb);
```

---

### 3.3 不同规则类型的构造参数

#### RuleB（闭合连接规则）
```cpp
RuleB(
    std::vector<int>& relations,      // [head, body1, body2, ...]
    std::vector<bool>& directions     // [dir1, dir2, ...]
)
```
**特点**：
- 头部变量：`h(X, Y)` - X 是第一个变量，Y 是最后一个变量
- 体部形成路径：`X → A → B → ... → Y`

---

#### RuleC（带 2 个常量的规则）
```cpp
RuleC(
    std::vector<int>& relations,      // [head, body1, body2, ...]
    std::vector<bool>& directions,    // [dir1, dir2, ...]
    bool& leftC,                      // 常量在左侧 or 右侧
    std::array<int, 2>& constants     // [常量1, 常量2]
)
```
**示例**：
```
h(X,c1) <= b1(X,c2)
leftC = false (c1 在右侧)
constants = [c1_id, c2_id]
```

---

#### RuleD（悬挂原子规则）
```cpp
RuleD(
    std::vector<int>& relations,      // [head, body1, body2, ...]
    std::vector<bool>& directions,    // [dir1, dir2, ...]
    bool& leftC,                      // 常量在左侧 or 右侧
    int constant                      // 常量 ID
)
```
**特点**：
- 头部有一个常量
- 体部没有常量（悬挂）
- **示例**：`h(X,c) <= b1(X,A), b2(A,B)`

---

#### RuleZ（零体规则）
```cpp
RuleZ(
    int& relation,                    // 头部关系 ID
    bool& leftC,                      // 常量在左侧 or 右侧
    int& constant                     // 常量 ID
)
```
**示例**：`h(X,c) <=` （没有 body）

---

#### RuleXXc（自环常量规则）
```cpp
RuleXXc(
    std::vector<int>& relations,      // [head, body]
    std::vector<bool>& directions,    // [dir]
    int& constant                     // 常量 ID
)
```
**示例**：`h(X,X) <= b(X,c)`

---

#### RuleXXd（自环悬挂规则）
```cpp
RuleXXd(
    std::vector<int>& relations,      // [head, body]
    std::vector<bool>& directions     // [dir]
)
```
**示例**：`h(X,X) <= b(X,A)`

---

#### RuleM（多规则组合）
```cpp
RuleM(
    std::vector<std::unique_ptr<Rule>>& memberRules  // 成员规则列表
)
```
**特点**：
- 不直接解析 relations/directions
- 而是包含多个子规则
- 所有子规则共享相同的 head
- **示例**：`h(X,Y) <= b1(X,A), b2(A,Y); b3(X,B), b4(B,Y)`

---

## 四、内部优化表示

### 为什么需要 `_relations` 和 `_directions`？

RuleB、RuleC、RuleD 都维护了内部优化版本：

```cpp
// RuleB 中：
std::vector<int> _relations;
std::vector<bool> _directions;
```

#### 原因：双向预测优化

知识图谱补全需要预测两个方向：
1. **Tail Query**：`h(X, ?)` - 给定头实体 X，预测尾实体
2. **Head Query**：`h(?, Y)` - 给定尾实体 Y，预测头实体

#### 举例说明

**原始规则**：`h(X,Y) <= b1(X,A), b2(A,Y)`

- **Tail Query** 时（给定 X，找 Y）：
  - 按原始顺序搜索：`X → A → Y`
  - 使用：`relations`, `directions`

- **Head Query** 时（给定 Y，找 X）：
  - 需要反向搜索：`Y → A → X`
  - 使用：`_relations`, `_directions`

#### `_relations` 和 `_directions` 如何生成？

```cpp
// RuleB 构造函数中：
this->_relations = relations;
std::reverse(_relations.begin()+1, _relations.end());  // 只反转 body 部分
// [h, b1, b2] → [h, b2, b1]

this->_directions = directions;
std::reverse(_directions.begin(), _directions.end());  // 反转方向
_directions.flip();  // 翻转每个布尔值
// [true, true] → [false, false]
```

**效果**：
```
原始规则：h(X,Y) <= b1(X,A), b2(A,Y)
         relations=[h,b1,b2], directions=[T,T]

反向规则：h(Y,X) <= b2(A,Y), b1(X,A)
        _relations=[h,b2,b1], _directions=[F,F]
```

这样在 `predictHeadQuery` 时，可以直接使用 `_relations` 和 `_directions` 进行高效搜索。

---

## 五、置信度计算

### 公式

```cpp
confidence = confWeight × (cpredicted / (predicted + numUnseen))
```

### 参数说明

- **cpredicted**：正确预测数（分子）
- **predicted**：总预测数（分母基础）
- **numUnseen**：拉普拉斯平滑（防止过拟合）
- **confWeight**：类型权重（不同规则类型有不同默认值）

### 示例

```cpp
// RuleB: 100 次预测，75 次正确
predicted = 100
cpredicted = 75
numUnseen = 5
confWeight = 1.0

confidence = 1.0 × (75 / (100 + 5)) = 0.714

// RuleZ: 20 次预测，15 次正确（零体规则置信度较低）
predicted = 20
cpredicted = 15
numUnseen = 5
confWeight = 0.01  // 零体规则权重很低

confidence = 0.01 × (15 / (20 + 5)) = 0.006
```

---

## 六、规则生命周期

### 1. 加载阶段（Loading）
```
字符串规则文件
    ↓
RuleStorage::readAnyTimeFormat()
    ↓
RuleFactory::parseAnytimeRule()
    ↓
创建具体规则对象（RuleB/RuleC/...）
    ↓
存储到 rules vector
```

### 2. 过滤阶段（Filtering）
```
RuleFactory::updateRules()
    ↓
检查配置（createRuleX, XminPreds, XminConf等）
    ↓
符合条件的规则加入 relToRules map
```

### 3. 应用阶段（Application）
```
Query: h(entity1, ?)
    ↓
根据关系 h 从 relToRules 获取相关规则
    ↓
对每个规则调用 predictTailQuery()
    ↓
DFS 搜索 + 收集候选实体
    ↓
按置信度排序返回结果
```

---

## 七、总结

### 核心设计思想

1. **继承层次**：抽象基类 + 具体实现类，支持多态
2. **工厂模式**：通过 RuleFactory 解析字符串创建规则
3. **双向优化**：维护正向和反向表示，支持高效双向查询
4. **统计驱动**：基于训练集统计计算置信度
5. **灵活配置**：通过阈值和权重控制规则行为

### 关键数据结构

- **relations**：关系 ID 序列（规则骨架）
- **directions**：原子方向（连接方式）
- **predicted/cpredicted**：统计信息（置信度计算）
- **confWeight/numUnseen**：平滑参数（防止过拟合）
- **type**：类型标识（运行时判断）

### 构造规则的本质

**从符号表示到计算表示的转换**：

```
符号： h(X,Y) <= b1(X,A), b2(A,Y)
        ↓ 解析
计算： relations=[id(h), id(b1), id(b2)]
       directions=[true, true]
       type="b"
```

这种表示方式使得规则可以在大规模知识图谱上高效执行！
