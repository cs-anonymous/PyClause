# RuleM Implementation Summary

## Overview
Successfully implemented **RuleM (Multi-Rule)** support for the PyClause rule-based knowledge graph completion system. RuleM represents rules with multiple parallel rule bodies sharing the same head, using **AND (intersection) semantics**.

## Rule Format
```
h(X,Y) <= body1 ; body2 ; body3
```

Where:
- Each body is separated by semicolon (`;`)
- All bodies share the same head relation `h(X,Y)`
- A prediction is valid only if **ALL** member rules predict it (intersection)

## Examples

### Unary RuleM (Length=1 bodies)
```
2341  13  0.0056  /award/award_nominee/award_nominations./award/award_nomination/award_nominee(/m/05yjhm,X) <= 
  /people/person/profession(X,/m/0d8qb) ; 
  /award/award_ceremony/awards_presented./award/award_honor/award_winner(/m/0jt3qpk,X)
```

### Binary RuleM (Length=2 bodies)
```
120  35  0.2917  /location/country/form_of_government(X,Y) <= 
  /location/statistical_region/places_exported_to./location/imports_and_exports/exported_to(A,X), /location/country/form_of_government(A,Y); 
  /location/location/adjoin_s./location/adjoining_relationship/adjoins(B,X), /location/country/form_of_government(B,Y)
```

## Implementation Details

### Files Modified

#### 1. Rule.h (Header)
- Added `RuleM` class declaration
- Private member: `std::vector<std::unique_ptr<Rule>> memberRules`
- Virtual methods: `predictHeadQuery`, `predictTailQuery`, `materialize`, `computeRuleString`, `predictTriple`
- Helper method: `intersectQueryResults` for result intersection

#### 2. Rule.cpp (Implementation)
- **Constructor**: Accepts vector of member rules, moves them into storage
- **Type identifier**: `"m"`
- **predictHeadQuery/predictTailQuery**: 
  - Call each member rule independently
  - Intersect results progressively
  - Apply filter set to final intersection
  - Early termination if any intersection becomes empty
- **materialize**: Intersect materialization results from all members
- **computeRuleString**: Reconstruct string with semicolon separators
- **predictTriple**: Check if ALL member rules predict the triple

#### 3. RuleFactory.h
- Added `parseMRule` method declaration
- Added `setCreateRuleM` setter
- Added configuration variables:
  - `bool createRuleM = true`
  - `int MnumUnseen = 0`
  - `int MminCorrect = 1`
  - `int MminPreds = 1`
  - `double MminConf = 0.00001`

#### 4. RuleFactory.cpp
- **parseAnytimeRule**: Check for semicolon first, delegate to `parseMRule` if found
- **parseMRule**:
  - Split bodies by `;`
  - Reconstruct individual rules: `head <= body_i`
  - Parse each as standard rule (no recursive RuleM)
  - Create RuleM with all member rules
  - Apply threshold checks (minPreds, minCorrect, minConf)
- **updateRules**: Added RuleM filtering logic
- **Setter methods**: Updated `setNumUnseen`, `setMinCorrect`, `setMinPred`, `setMinConf`, added `setCreateRuleM`

#### 5. Loader.cpp
- **setRuleOptions**: Added configuration handlers:
  - `load_m_rules` → `setCreateRuleM`
  - `m_num_unseen` → `setNumUnseen`
  - `m_min_support` → `setMinCorrect`
  - `m_min_preds` → `setMinPred`
  - `m_min_conf` → `setMinConf`

#### 6. config-default.yaml
- Added RuleM configuration section:
```yaml
## M rules (Multi-Rule)
load_m_rules: True
m_num_unseen: 5
m_min_support: -1
m_min_preds: -1
m_min_conf: 0.0001
```

## Configuration Usage

### Python Example
```python
from clause import Options
from c_clause import Loader

opts = Options()

# Enable RuleM loading
opts.set("loader.load_m_rules", True)

# Set thresholds
opts.set("loader.m_min_conf", 0.2)
opts.set("loader.m_min_support", 10)
opts.set("loader.m_min_preds", 15)
opts.set("loader.m_num_unseen", 5)

loader = Loader(options=opts.get("loader"))
loader.load_data(data=train_data)
loader.load_rules(rules=rule_file)
```

### Disabling RuleM
```python
opts.set("loader.load_m_rules", False)
```

## Key Features

### 1. Intersection Semantics
- RuleM requires **ALL** member rules to agree
- More conservative than OR-based aggregation
- Reduces false positives for high-confidence multi-path reasoning

### 2. Efficient Early Termination
- Stops processing if any intermediate intersection is empty
- Progressive filtering reduces computational overhead

### 3. Seamless Integration
- Works with existing rule types (B, C, D, Z, XXc, XXd)
- Compatible with all handlers (QA, Ranking, Prediction)
- Respects filtering and configuration options

### 4. Flexible Composition
- Supports any combination of member rule types
- No limit on number of parallel bodies
- Each member can have different lengths

## Parsing Logic

1. **Detection**: Check for `;` in rule string
2. **Split**: Separate by `;` into body parts
3. **Reconstruction**: Create individual rules with shared head
4. **Validation**: Each member must parse successfully
5. **Creation**: Assemble RuleM from validated members
6. **Threshold Check**: Apply minPreds, minCorrect, minConf

## Prediction Workflow

### Query-Based Prediction (Head/Tail)
```
Input: query + memberRules
↓
For each memberRule:
  - Get predictions → tempResults
  - Intersect with previous → intersected
  - Early exit if empty
↓
Apply filterSet to final intersection
↓
Output: candidates predicted by ALL rules
```

### Materialization
```
Input: triples + memberRules
↓
Get materialization from rule1 → tempPreds1
↓
For rule2...ruleN:
  - Get materialization → tempPreds2
  - Intersect tempPreds1 ∩ tempPreds2
  - Early exit if empty
↓
Output: triples predicted by ALL rules
```

## Testing

Run test script:
```bash
cd c:\Users\sy650\IdeaProjects\Tarmorn
python test_rulem.py
```

Expected output:
- Successful rule loading
- Correct rule string serialization
- Multi-body rules properly indexed

## Compilation

Automatic via pip:
```bash
cd PyClause
pip install . --force-reinstall
```

Or with development mode:
```bash
pip install -e .
```

## Benefits

1. **Precision**: Intersection reduces noise from individual weak rules
2. **Interpretability**: Multiple supporting paths increase confidence
3. **Flexibility**: Compose complex patterns from simpler rules
4. **Compatibility**: Integrates seamlessly with existing codebase

## Limitations

1. **Stricter**: May miss valid predictions if ANY member fails
2. **Computation**: Requires running all member rules (no short-circuit on success)
3. **Memory**: Maintains temporary result sets for intersection

## Future Enhancements

- Weighted intersection (require k-of-n members)
- Performance optimization for large member sets
- Support for mixed AND/OR compositions
- Specialized indexing for RuleM lookup

---

**Status**: ✅ Implementation Complete and Tested
**Type Identifier**: `"m"`
**Configuration Prefix**: `m_*`
**Default**: Enabled (`load_m_rules: True`)
