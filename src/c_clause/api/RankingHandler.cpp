#include "RankingHandler.h"


#include <string>
#include <map>
#include <functional>
#include <unordered_set>
#include <sstream>
#include <cmath>
#include <numeric>
#include <fstream>
#include <algorithm>


RankingHandler::RankingHandler(std::map<std::string, std::string> options): BackendHandler(){
    auto verb = options.find("verbose");
    if (verb!=options.end()){
        this->verbose = util::stringToBool(verb->second);
    }
    setRankingOptions(options, ranker);
    setOptions(options);
}


void RankingHandler::setOptions(std::map<std::string, std::string> options){
    // register options for ranker

     struct OptionHandler {
        std::string name;
        std::function<void(std::string)> setter;
    };
    
    std::vector<OptionHandler> handlers = {
        {"collect_rules", [this](std::string val) {this->setCollectRules(util::stringToBool(val));}},
    };

    for (auto& handler : handlers) {
        auto opt = options.find(handler.name);
        if (opt != options.end()) {
            if (verbose){
                std::cout<< "Setting option "<<handler.name<<" to: "<<opt->second<<std::endl;
            }
            handler.setter(opt->second);
        }
    }
}

void RankingHandler::setOptionsFrontend(std::map<std::string, std::string> options){
    setRankingOptions(options, ranker);
    setOptions(options);
}


void RankingHandler::calculateRanking(std::shared_ptr<Loader> dHandler){
    index = dHandler->getIndex();
    myDhandler = dHandler;
    if (dHandler->getXGBoostModelHandle() && !ranker.hasXGBoostModel()){
        ranker.setXGBoostModelHandle(dHandler->getXGBoostModelHandle());
    }
    ranker.clearAll();
    if (collectRules){
        ranker.setSaveCandidateRules(true);
    }
    ranker.makeRanking(dHandler->getTarget(), dHandler->getData(), dHandler->getRules(),dHandler->getFilter());
}


void RankingHandler::writeRanking(std::string writePath, std::shared_ptr<Loader> dHandler){
    ranker.writeRanking(dHandler->getTarget(), writePath);

}

void RankingHandler::writeRules(std::string writePath, std::shared_ptr<Loader> dHandler, std::string direction, bool strings){
    if (collectRules){
        ranker.writeRules(dHandler->getTarget(), writePath, direction, strings);
    }else{
        throw std::runtime_error("Please set 'ranking_handler.collect_rules' to true when you want to write the rules.");
    }
}

void RankingHandler::loadXGBoostModel(std::string modelPath){
    ranker.loadXGBoostModel(modelPath);
}

namespace {
    std::string escapeCsv(const std::string& value) {
        bool needsQuotes = value.find_first_of(",\"\n\r") != std::string::npos;
        if (!needsQuotes) {
            return value;
        }
        std::string out;
        out.reserve(value.size() + 2);
        out.push_back('"');
        for (char c : value) {
            if (c == '"') {
                out.push_back('"');
                out.push_back('"');
            } else {
                out.push_back(c);
            }
        }
        out.push_back('"');
        return out;
    }

    struct DepGraphFeatures {
        int num_rules = 0;
        double w_max = 0.0;
        double w_second_max = 0.0;
        double w_top1_top2_gap = 0.0;
        double w_std = 0.0;
        double w_top3_sum = 0.0;
        double w_top5_sum = 0.0;

        int num_neg_edges = 0;
        int num_rules_with_neg_incoming = 0;
        int num_rules_with_neg_outgoing = 0;
        int max_neg_indegree = 0;
        double mean_neg_indegree = 0.0;
        int max_neg_outdegree = 0;
        int num_neg_components = 0;
        int num_rules_neg_dedup = 0;
        int num_neg_component = 0;
        int largest_neg_component_size = 0;

        int num_pos_edges = 0;
        int num_rules_with_pos_edges = 0;
        int num_pos_components = 0;
        int largest_pos_component_size = 0;
        int max_neg_outdegree_minus_indegree = 0;
        int max_pos_outdegree = 0;
        int outdegree_of_top_rule = 0;
        int indegree_of_top_rule = 0;

        double score_noisyor = 0.0;
        double score_maxplus = 0.0;
        double score_expdecay_tau_0_25 = 0.0;
        double score_expdecay_tau_0_5 = 0.0;
        double score_expdecay_tau_1 = 0.0;
        double score_expdecay_tau_2 = 0.0;
        double score_expdecay_tau_4 = 0.0;

        int num_lift_pos_gt_1 = 0;
        int num_lift_neg_gt_0_5 = 0;
        double max_lift_neg = 0.0;
        double max_lift_pos = 0.0;
        double sum_lift_pos = 0.0;
        double sum_lift_neg = 0.0;
        double sum_top3_lift_pos = 0.0;
        double sum_top3_lift_neg = 0.0;
    };

    struct EdgeInfo {
        int srcIdx;
        int tgtIdx;
        double absLift;
    };

    void computeComponents(
        int numRules,
        const std::vector<std::vector<int>>& adj,
        const std::vector<char>& nodeHasEdge,
        int& numComponents,
        int& largestSize
    ) {
        numComponents = 0;
        largestSize = 0;
        std::vector<char> visited(numRules, 0);
        for (int i = 0; i < numRules; i++) {
            if (!nodeHasEdge[i] || visited[i]) {
                continue;
            }
            int size = 0;
            std::vector<int> stack;
            stack.push_back(i);
            visited[i] = 1;
            while (!stack.empty()) {
                int curr = stack.back();
                stack.pop_back();
                size += 1;
                for (int nxt : adj[curr]) {
                    if (!visited[nxt]) {
                        visited[nxt] = 1;
                        stack.push_back(nxt);
                    }
                }
            }
            numComponents += 1;
            if (size > largestSize) {
                largestSize = size;
            }
        }
    }

    DepGraphFeatures extractFeatures(const std::vector<Rule*>& rules) {
        DepGraphFeatures feat;
        if (rules.empty()) {
            return feat;
        }

        std::unordered_map<int, Rule*> appliedById;
        appliedById.reserve(rules.size());
        std::vector<Rule*> appliedRules;
        appliedRules.reserve(rules.size());
        for (Rule* rule : rules) {
            if (!rule) {
                continue;
            }
            int rid = rule->getID();
            if (appliedById.find(rid) == appliedById.end()) {
                appliedById[rid] = rule;
                appliedRules.push_back(rule);
            }
        }

        int numRules = static_cast<int>(appliedRules.size());
        feat.num_rules = numRules;
        if (numRules == 0) {
            return feat;
        }

        std::vector<double> confs;
        confs.reserve(numRules);
        int topRuleIdx = 0;
        double topConf = -1.0;
        for (int i = 0; i < numRules; i++) {
            double conf = appliedRules[i]->getConfidence();
            confs.push_back(conf);
            if (conf > topConf) {
                topConf = conf;
                topRuleIdx = i;
            }
        }

        std::vector<double> confsSorted = confs;
        std::sort(confsSorted.begin(), confsSorted.end(), std::greater<double>());
        feat.w_max = confsSorted[0];
        feat.score_maxplus = feat.w_max;
        if (confsSorted.size() > 1) {
            feat.w_second_max = confsSorted[1];
        }
        feat.w_top1_top2_gap = feat.w_max - feat.w_second_max;

        double mean = std::accumulate(confs.begin(), confs.end(), 0.0) / static_cast<double>(numRules);
        double varSum = 0.0;
        for (double v : confs) {
            double diff = v - mean;
            varSum += diff * diff;
        }
        feat.w_std = std::sqrt(varSum / static_cast<double>(numRules));

        for (size_t i = 0; i < confsSorted.size() && i < 3; i++) {
            feat.w_top3_sum += confsSorted[i];
        }
        for (size_t i = 0; i < confsSorted.size() && i < 5; i++) {
            feat.w_top5_sum += confsSorted[i];
        }

        std::unordered_map<int, int> idToIdx;
        idToIdx.reserve(numRules);
        for (int i = 0; i < numRules; i++) {
            idToIdx[appliedRules[i]->getID()] = i;
        }

        std::vector<int> negIn(numRules, 0);
        std::vector<int> negOut(numRules, 0);
        std::vector<int> posIn(numRules, 0);
        std::vector<int> posOut(numRules, 0);
        std::vector<EdgeInfo> negEdges;
        std::vector<EdgeInfo> posEdges;
        negEdges.reserve(numRules);
        posEdges.reserve(numRules);

        for (int i = 0; i < numRules; i++) {
            Rule* rule = appliedRules[i];
            int rid = rule->getID();
            for (const auto& depPair : rule->dependency) {
                int otherId = depPair.first;
                Dependency* dep = depPair.second;
                if (!dep || dep->i != rid) {
                    continue;
                }
                auto itOther = idToIdx.find(otherId);
                if (itOther == idToIdx.end()) {
                    continue;
                }
                int j = itOther->second;
                if (dep->lift > 0) {
                    posOut[i] += 1;
                    posIn[j] += 1;
                    posEdges.push_back({i, j, std::abs(dep->lift)});
                } else if (dep->lift < 0) {
                    negOut[i] += 1;
                    negIn[j] += 1;
                    negEdges.push_back({i, j, std::abs(dep->lift)});
                }
            }
        }

        feat.num_neg_edges = static_cast<int>(negEdges.size());
        feat.num_pos_edges = static_cast<int>(posEdges.size());

        for (int i = 0; i < numRules; i++) {
            if (negIn[i] > 0) {
                feat.num_rules_with_neg_incoming += 1;
            }
            if (negOut[i] > 0) {
                feat.num_rules_with_neg_outgoing += 1;
            }
            if (negIn[i] > feat.max_neg_indegree) {
                feat.max_neg_indegree = negIn[i];
            }
            if (negOut[i] > feat.max_neg_outdegree) {
                feat.max_neg_outdegree = negOut[i];
            }
            int negDiff = negOut[i] - negIn[i];
            if (negDiff > feat.max_neg_outdegree_minus_indegree) {
                feat.max_neg_outdegree_minus_indegree = negDiff;
            }
            if ((posIn[i] + posOut[i]) > 0) {
                feat.num_rules_with_pos_edges += 1;
            }
            if (posOut[i] > feat.max_pos_outdegree) {
                feat.max_pos_outdegree = posOut[i];
            }
        }
        if (numRules > 0) {
            double negInSum = 0.0;
            for (int v : negIn) {
                negInSum += v;
            }
            feat.mean_neg_indegree = negInSum / static_cast<double>(numRules);
        }

        feat.outdegree_of_top_rule = negOut[topRuleIdx] + posOut[topRuleIdx];
        feat.indegree_of_top_rule = negIn[topRuleIdx] + posIn[topRuleIdx];

        if (!negEdges.empty()) {
            std::vector<std::vector<int>> adjNeg(numRules);
            std::vector<char> nodeHasNeg(numRules, 0);
            for (const auto& e : negEdges) {
                adjNeg[e.srcIdx].push_back(e.tgtIdx);
                adjNeg[e.tgtIdx].push_back(e.srcIdx);
                nodeHasNeg[e.srcIdx] = 1;
                nodeHasNeg[e.tgtIdx] = 1;
            }
            computeComponents(numRules, adjNeg, nodeHasNeg, feat.num_neg_components, feat.largest_neg_component_size);
        }
        feat.num_neg_component = feat.num_neg_components;

        if (!posEdges.empty()) {
            std::vector<std::vector<int>> adjPos(numRules);
            std::vector<char> nodeHasPos(numRules, 0);
            for (const auto& e : posEdges) {
                adjPos[e.srcIdx].push_back(e.tgtIdx);
                adjPos[e.tgtIdx].push_back(e.srcIdx);
                nodeHasPos[e.srcIdx] = 1;
                nodeHasPos[e.tgtIdx] = 1;
            }
            computeComponents(numRules, adjPos, nodeHasPos, feat.num_pos_components, feat.largest_pos_component_size);
        }

        std::vector<char> validNodes(numRules, 1);
        if (!negEdges.empty()) {
            std::vector<EdgeInfo> negEdgesSorted = negEdges;
            std::sort(negEdgesSorted.begin(), negEdgesSorted.end(), [](const EdgeInfo& a, const EdgeInfo& b) {
                return a.absLift > b.absLift;
            });
            for (const auto& e : negEdgesSorted) {
                if (validNodes[e.srcIdx]) {
                    validNodes[e.tgtIdx] = 0;
                }
            }
        }
        int validCount = 0;
        for (char v : validNodes) {
            if (v) {
                validCount += 1;
            }
        }
        feat.num_rules_neg_dedup = validCount;

        auto toSurprisal = [](double conf) {
            if (conf <= 0.0) {
                return 0.0;
            }
            if (conf >= 1.0) {
                conf = 1.0 - 1e-12;
            }
            return -std::log(1.0 - conf);
        };
        std::vector<double> surprisals;
        surprisals.reserve(numRules);
        for (double conf : confs) {
            double s = toSurprisal(conf);
            if (s > 0.0) {
                surprisals.push_back(s);
            }
        }
        std::sort(surprisals.begin(), surprisals.end(), std::greater<double>());
        auto scoreExpDecay = [&surprisals](double tau) {
            double sum = 0.0;
            for (size_t i = 0; i < surprisals.size(); i++) {
                sum += surprisals[i] * std::exp(-tau * static_cast<double>(i));
            }
            return 1.0 - std::exp(-sum);
        };
        feat.score_noisyor = scoreExpDecay(0.0);
        feat.score_expdecay_tau_0_25 = scoreExpDecay(0.25);
        feat.score_expdecay_tau_0_5 = scoreExpDecay(0.5);
        feat.score_expdecay_tau_1 = scoreExpDecay(1.0);
        feat.score_expdecay_tau_2 = scoreExpDecay(2.0);
        feat.score_expdecay_tau_4 = scoreExpDecay(4.0);

        std::vector<double> posLifts;
        std::vector<double> negLifts;
        posLifts.reserve(posEdges.size());
        negLifts.reserve(negEdges.size());
        for (const auto& e : posEdges) {
            posLifts.push_back(e.absLift);
            feat.sum_lift_pos += e.absLift;
            if (e.absLift >= 1.0) {
                feat.num_lift_pos_gt_1 += 1;
            }
            if (e.absLift > feat.max_lift_pos) {
                feat.max_lift_pos = e.absLift;
            }
        }
        for (const auto& e : negEdges) {
            negLifts.push_back(e.absLift);
            feat.sum_lift_neg += e.absLift;
            if (e.absLift >= 0.5) {
                feat.num_lift_neg_gt_0_5 += 1;
            }
            if (e.absLift > feat.max_lift_neg) {
                feat.max_lift_neg = e.absLift;
            }
        }

        std::sort(posLifts.begin(), posLifts.end(), std::greater<double>());
        std::sort(negLifts.begin(), negLifts.end(), std::greater<double>());
        for (size_t i = 0; i < posLifts.size() && i < 3; i++) {
            feat.sum_top3_lift_pos += posLifts[i];
        }
        for (size_t i = 0; i < negLifts.size() && i < 3; i++) {
            feat.sum_top3_lift_neg += negLifts[i];
        }

        return feat;
    }
}

void RankingHandler::saveDependencyGraph(std::string writePath){
    if (!collectRules){
        throw std::runtime_error("Please set 'ranking_handler.collect_rules' to true when you want to save dependency graph features.");
    }
    if (!myDhandler){
        throw std::runtime_error("Please call calculate_ranking() before save_dependency_graph().");
    }
    if (!index){
        index = myDhandler->getIndex();
    }

    std::ofstream file(writePath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to create file. Please check if the paths are correct: " + writePath);
    }

    file
        << "query,candidate,label,"
        << "num_rules,w_max,w_second_max,w_top1_top2_gap,w_std,w_top3_sum,w_top5_sum,"
        << "num_neg_edges,num_rules_with_neg_incoming,num_rules_with_neg_outgoing,max_neg_indegree,mean_neg_indegree,max_neg_outdegree,"
        << "num_neg_components,num_rules_neg_dedup,num_neg_component,largest_neg_component_size,"
        << "num_pos_edges,num_rules_with_pos_edges,num_pos_components,largest_pos_component_size,"
        << "max_neg_outdegree_minus_indegree,max_pos_outdegree,outdegree_of_top_rule,indegree_of_top_rule,"
        << "score_noisyor,score_maxplus,score_expdecay_tau_0.25,score_expdecay_tau_0.5,score_expdecay_tau_1,score_expdecay_tau_2,score_expdecay_tau_4,"
        << "num_lift_pos_gt_1,num_lift_neg_gt_0.5,max_lift_neg,max_lift_pos,sum_lift_pos,sum_lift_neg,sum_top3_lift_pos,sum_top3_lift_neg\n";

    auto& target = myDhandler->getTarget();

    auto writeRow = [&file](
        const std::string& query,
        const std::string& candidate,
        int label,
        const DepGraphFeatures& f
    ) {
        file << escapeCsv(query) << "," << escapeCsv(candidate) << "," << label << ","
             << f.num_rules << "," << f.w_max << "," << f.w_second_max << "," << f.w_top1_top2_gap << ","
             << f.w_std << "," << f.w_top3_sum << "," << f.w_top5_sum << ","
             << f.num_neg_edges << "," << f.num_rules_with_neg_incoming << "," << f.num_rules_with_neg_outgoing << ","
             << f.max_neg_indegree << "," << f.mean_neg_indegree << "," << f.max_neg_outdegree << ","
             << f.num_neg_components << "," << f.num_rules_neg_dedup << "," << f.num_neg_component << "," << f.largest_neg_component_size << ","
             << f.num_pos_edges << "," << f.num_rules_with_pos_edges << "," << f.num_pos_components << "," << f.largest_pos_component_size << ","
             << f.max_neg_outdegree_minus_indegree << "," << f.max_pos_outdegree << "," << f.outdegree_of_top_rule << "," << f.indegree_of_top_rule << ","
             << f.score_noisyor << "," << f.score_maxplus << "," << f.score_expdecay_tau_0_25 << "," << f.score_expdecay_tau_0_5 << ","
             << f.score_expdecay_tau_1 << "," << f.score_expdecay_tau_2 << "," << f.score_expdecay_tau_4 << ","
             << f.num_lift_pos_gt_1 << "," << f.num_lift_neg_gt_0_5 << "," << f.max_lift_neg << "," << f.max_lift_pos << ","
             << f.sum_lift_pos << "," << f.sum_lift_neg << "," << f.sum_top3_lift_pos << "," << f.sum_top3_lift_neg
             << "\n";
    };

    std::size_t rowsWritten = 0;
    const std::size_t logEvery = 10000;

    auto processDirection = [&](bool dirIsTail){
        auto& data = dirIsTail ? ranker.getTailQcandsRules() : ranker.getHeadQcandsRules();
        for (const auto& relPair : data){
            int rel = relPair.first;
            const std::string relStr = index->getStringOfRelId(rel);
            for (const auto& srcPair : relPair.second){
                int source = srcPair.first;

                int* begin = nullptr;
                int length = 0;
                if (dirIsTail){
                    target.getTforHR(source, rel, begin, length);
                } else {
                    target.getHforTR(source, rel, begin, length);
                }
                std::unordered_set<int> gtSet;
                gtSet.reserve(length);
                for (int i = 0; i < length; i++) {
                    gtSet.insert(begin[i]);
                }

                double globalMaxConf = -1.0;
                for (const auto& candPair : srcPair.second){
                    std::unordered_set<int> seen;
                    for (Rule* rule : candPair.second){
                        int rid = rule->getID();
                        if (seen.insert(rid).second){
                            double conf = rule->getConfidence();
                            if (conf > globalMaxConf){
                                globalMaxConf = conf;
                            }
                        }
                    }
                }

                bool hasGtCandidate = false;
                std::vector<int> filteredCandidates;
                filteredCandidates.reserve(srcPair.second.size());
                for (const auto& candPair : srcPair.second){
                    int cand = candPair.first;
                    std::unordered_set<int> seen;
                    int uniqueRules = 0;
                    double candMaxConf = -1.0;
                    for (Rule* rule : candPair.second){
                        int rid = rule->getID();
                        if (seen.insert(rid).second){
                            uniqueRules += 1;
                            double conf = rule->getConfidence();
                            if (conf > candMaxConf){
                                candMaxConf = conf;
                            }
                        }
                    }
                    if (uniqueRules == 1 && candMaxConf < globalMaxConf){
                        continue;
                    }
                    if (gtSet.find(cand) != gtSet.end()){
                        hasGtCandidate = true;
                    }
                    filteredCandidates.push_back(cand);
                }

                if (!hasGtCandidate){
                    continue;
                }

                for (int cand : filteredCandidates){
                    const std::string candStr = index->getStringOfNodeId(cand);
                    std::string query;
                    if (dirIsTail){
                        const std::string srcStr = index->getStringOfNodeId(source);
                        query = srcStr + " " + relStr + " ?";
                    } else {
                        const std::string srcStr = index->getStringOfNodeId(source);
                        query = "? " + relStr + " " + srcStr;
                    }
                    int label = (gtSet.find(cand) != gtSet.end()) ? 1 : 0;
                    DepGraphFeatures features = extractFeatures(srcPair.second.at(cand));
                    writeRow(query, candStr, label, features);
                    rowsWritten += 1;
                    if (rowsWritten % logEvery == 0){
                        std::cout << "saveDependencyGraph: written " << rowsWritten << " rows..." << std::endl;
                    }
                }
            }
        }
    };

    processDirection(true);
    processDirection(false);

    file.close();
    std::cout << "saveDependencyGraph: written " << rowsWritten << " total rows." << std::endl;
    std::cout << "Dependency graph features written to:  " + writePath << std::endl;
}


std::unordered_map<int,std::unordered_map<int,std::vector<std::pair<int, double>>>> RankingHandler::getRanking(std::string headOrTail){
    if (headOrTail=="head"){
        return ranker.getHeadQcandsConfs();
    }else if (headOrTail=="tail"){
        return ranker.getTailQcandsConfs();
    }else{
        throw std::runtime_error("Please specify 'head' or 'tail' as first argument of getRanking");
    }
}


 std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::pair<std::string, double>>>>RankingHandler::getStrRanking(std::string headOrTail) {
    auto idxRanking = (headOrTail == "head") ? ranker.getHeadQcandsConfs() : ranker.getTailQcandsConfs();
    if (!(headOrTail =="head") && !(headOrTail =="tail")){
        throw std::runtime_error("Please specify 'head' or 'tail' as first argument of getRanking");
    }
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::pair<std::string, double>>>> strRanking;
    for (const auto& outer_pair : idxRanking) {
        const std::string outerKeyStr = index->getStringOfRelId(outer_pair.first);
        for (const auto& middle_pair : outer_pair.second) {
            const std::string middleKeyStr = index->getStringOfNodeId(middle_pair.first);
            std::vector<std::pair<std::string, double>>& pairsVec = strRanking[outerKeyStr][middleKeyStr];
            for (const auto& inner_pair : middle_pair.second) {
                const std::string innerKeyStr = index->getStringOfNodeId(inner_pair.first);
                pairsVec.emplace_back(innerKeyStr, inner_pair.second);
            }
        }
    }
    return strRanking;
 }


std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::vector<int>>>> RankingHandler::getIdxRules(std::string headOrTail) {
    if (!collectRules){
        throw std::runtime_error("The handler option 'collect_rules' is set to false. Recreate the handler with the option set to true.");
    }

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::vector<Rule*>>>>& data = 
     (headOrTail == "head") ? ranker.getHeadQcandsRules(): ranker.getTailQcandsRules();

    if (!(headOrTail =="head") && !(headOrTail =="tail")){
        throw std::runtime_error("Please specify 'head' or 'tail' as first argument of getRanking");
    }

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::vector<int>>>> convertedData;

    // relations
    for (const auto &outerPair : data) {
        int outerKey = outerPair.first;
        const auto &middleMap = outerPair.second;

        // source entities
        for (const auto &middlePair : middleMap) {
            int middleKey = middlePair.first;
            const auto &innerMap = middlePair.second;

            // candidates + rules
            for (const auto &innerPair : innerMap) {
                int innerKey = innerPair.first;
                const std::vector<Rule*>& rules = innerPair.second;
                std::vector<int> ruleIds;
                ruleIds.reserve(rules.size());

                for (Rule* rule : rules) {
                    if (rule != nullptr) {
                        ruleIds.push_back(rule->getID());
                    }else{
                        throw std::runtime_error("Rule object does not exist. This should not happen and is an internal error.");
                    }
                }

                convertedData[outerKey][middleKey][innerKey] = ruleIds;
            }
        }
    }
    return convertedData;
}


std::unordered_map<std::string,std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>>> RankingHandler::getStrRules(std::string headOrTail) {
    if (!collectRules){
        throw std::runtime_error("The handler option 'collect_rules' is set to false. Recreate the handler with the option set to true.");
    }

    std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::vector<Rule*>>>>& data = 
     (headOrTail == "head") ? ranker.getHeadQcandsRules(): ranker.getTailQcandsRules();

    if (!(headOrTail =="head") && !(headOrTail =="tail")){
        throw std::runtime_error("Please specify 'head' or 'tail' as first argument of getRanking");
    }

    std::unordered_map<std::string,std::unordered_map<std::string, std::unordered_map<std::string, std::vector<std::string>>>> convertedData;

    // relations
    for (const auto &outerPair : data) {
        int outerKey = outerPair.first;
        std::string rel = index->getStringOfRelId(outerKey);
        const auto &middleMap = outerPair.second;

        // source entities
        for (const auto &middlePair : middleMap) {
            int middleKey = middlePair.first;
            std::string source = index->getStringOfNodeId(middleKey);
            const auto &innerMap = middlePair.second;

            // candidates + rules
            for (const auto &innerPair : innerMap) {
                int innerKey = innerPair.first;
                std::string cand = index->getStringOfNodeId(innerKey);
                const std::vector<Rule*>& rules = innerPair.second;
                std::vector<std::string> ruleIds;
                ruleIds.reserve(rules.size());

                for (Rule* rule : rules) {
                    if (rule != nullptr) {
                        ruleIds.push_back(rule->computeRuleString(index.get()));
                    }else{
                        throw std::runtime_error("Rule object does not exist. This should not happen and is an internal error.");
                    }
                }
                convertedData[rel][source][cand] = ruleIds;
            }
        }
    }
    return convertedData;
}





 void RankingHandler::setCollectRules(bool ind){
    collectRules = ind;
 }
