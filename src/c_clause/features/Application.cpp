#include <map>
#include <unordered_map>
#include <unordered_set>
#include <omp.h>
#include <memory>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <functional>
#include <chrono>
#include <cmath>
#include <sstream>
#include <numeric>
#include <limits>
#include <atomic>
#include <mutex>


#include "Application.h"
#include "../core/TripleStorage.h"
#include "../core/RuleStorage.h"
#include "../core/Types.h"
#include "../core/Rule.h"
#include "../core/Globals.h"
#include "xgboost/c_api.h"





void ApplicationHandler::calculateTripleScores(std::vector<Triple> triples, TripleStorage& train, RuleStorage& rules){

    tripleScores.resize(triples.size());
    if (score_collectGr){
        tripleGroundings.resize(triples.size());
    }


    typedef void (ApplicationHandler::*SortAndProcessPtr)(std::vector<std::pair<int,double>>&, QueryResults&, TripleStorage&);
    SortAndProcessPtr sortAndProcess = nullptr;

    if(rank_aggrFunc=="noisyor") {
        sortAndProcess = &ApplicationHandler::sortAndProcessNoisy;
    } else if (rank_aggrFunc=="maxplus") {
        sortAndProcess = &ApplicationHandler::sortAndProcessMax;
    }else{
        throw std::runtime_error("Dont understand the aggregation function.");
    }

    #pragma omp parallel num_threads(num_thr)
    {
        QueryResults tripleResults(1, 1);
        // we dont need to set num_top_rules as the stopping is handled outside; there is only one "candidate"
        tripleResults.setAggrFunc(rank_aggrFunc);
        RuleGroundings ruleGroundings;   
        #pragma omp for schedule(dynamic)
        for (int i=0; i<triples.size(); i++){
           
            if (verbose && i%1000==0 && i>0){
                std::cout<<"Scored "<<(i/1000) * 1000<<" triples..."<<std::endl;
            }
            Triple triple = triples[i];
            int head = triple[0];
            int rel = triple[1];
            int tail = triple[2];
            auto& relRules = rules.getRelRules(rel);
            
                
            //triple = head, rel, tail
            int ctr = 0;
            for (Rule* rule: relRules){
                bool madePred;
                if (score_collectGr){
                    madePred = rule->predictTriple(head, tail, train, tripleResults, &ruleGroundings);
                }else{
                    madePred = rule->predictTriple(head, tail, train, tripleResults, nullptr);
                }
                        
                if (madePred){
                    ctr+= 1;
                }
                if (ctr>=score_numTopRules && score_numTopRules>0){
                    break;
                }
                // potentially this is possible for scoring as maxplus scores are currently the same as max scores
                // however, for groundings tracking we want the num_top_rules parameter to have effect
                // if (rank_aggrFunc=="maxplus" && ctr==1){
                //     break;
                // }

            }

            // we actually only have on candidate but we still need to process
            std::unordered_map<int, double>& candScores = tripleResults.getCandScores();
            std::vector<std::pair<int, double>> sortedCandScores(candScores.begin(), candScores.end());
            // tie handling, final processing, sorting
            (this->*sortAndProcess)(sortedCandScores, tripleResults, train);

            #pragma omp critical
            {  
                double trScore = 0;
                if (sortedCandScores.size()>0){
                    trScore = sortedCandScores[0].second;
                }
                // for easy conversion later
                tripleScores.at(i) = { (double) triple[0],  (double) triple[1], (double) triple[2], trScore};  
                if (score_collectGr){
                    tripleGroundings.at(i) = std::make_pair(triple, ruleGroundings);
                }    
            }
            tripleResults.clear();
            ruleGroundings.clear();
        }
    }
}

namespace {
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

    DepGraphFeatures extractFeaturesFromRules(const std::vector<Rule*>& rules) {
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

    void fillFeatureRow(const DepGraphFeatures& f, std::array<float, 40>& out) {
        out = {
            static_cast<float>(f.num_rules),
            static_cast<float>(f.w_max),
            static_cast<float>(f.w_second_max),
            static_cast<float>(f.w_top1_top2_gap),
            static_cast<float>(f.w_std),
            static_cast<float>(f.w_top3_sum),
            static_cast<float>(f.w_top5_sum),

            static_cast<float>(f.num_neg_edges),
            static_cast<float>(f.num_rules_with_neg_incoming),
            static_cast<float>(f.num_rules_with_neg_outgoing),
            static_cast<float>(f.max_neg_indegree),
            static_cast<float>(f.mean_neg_indegree),
            static_cast<float>(f.max_neg_outdegree),
            static_cast<float>(f.num_neg_components),
            static_cast<float>(f.num_rules_neg_dedup),
            static_cast<float>(f.num_neg_component),
            static_cast<float>(f.largest_neg_component_size),

            static_cast<float>(f.num_pos_edges),
            static_cast<float>(f.num_rules_with_pos_edges),
            static_cast<float>(f.num_pos_components),
            static_cast<float>(f.largest_pos_component_size),
            static_cast<float>(f.max_neg_outdegree_minus_indegree),
            static_cast<float>(f.max_pos_outdegree),
            static_cast<float>(f.outdegree_of_top_rule),
            static_cast<float>(f.indegree_of_top_rule),

            static_cast<float>(f.score_noisyor),
            static_cast<float>(f.score_maxplus),
            static_cast<float>(f.score_expdecay_tau_0_25),
            static_cast<float>(f.score_expdecay_tau_0_5),
            static_cast<float>(f.score_expdecay_tau_1),
            static_cast<float>(f.score_expdecay_tau_2),
            static_cast<float>(f.score_expdecay_tau_4),

            static_cast<float>(f.num_lift_pos_gt_1),
            static_cast<float>(f.num_lift_neg_gt_0_5),
            static_cast<float>(f.max_lift_neg),
            static_cast<float>(f.max_lift_pos),
            static_cast<float>(f.sum_lift_pos),
            static_cast<float>(f.sum_lift_neg),
            static_cast<float>(f.sum_top3_lift_pos),
            static_cast<float>(f.sum_top3_lift_neg)
        };
    }
}

void ApplicationHandler::calculateQueryResults(TripleStorage& target, TripleStorage& train, RuleStorage& rules, TripleStorage& addFilter, bool dirIsTail){
    // define rule prediction function depending on direction
    typedef bool (Rule::*RulePredFunc)(int, TripleStorage&, QueryResults&, ManySet);
    RulePredFunc predictHeadOrTail;

    if(dirIsTail){
        predictHeadOrTail = &Rule::predictTailQuery;
    }else{
        predictHeadOrTail = &Rule::predictHeadQuery;
    }

    typedef void (ApplicationHandler::*SortAndProcessPtr)(std::vector<std::pair<int,double>>&, QueryResults&, TripleStorage&);
    SortAndProcessPtr sortAndProcess = nullptr;

    if (rank_aggrFunc == "xgboost") {
        sortAndProcess = &ApplicationHandler::sortAndProcessXGBoost;
    } else if(rank_aggrFunc=="noisyor") {
        sortAndProcess = &ApplicationHandler::sortAndProcessNoisy;
    } else if (rank_aggrFunc=="maxplus") {
        sortAndProcess = &ApplicationHandler::sortAndProcessMax;
    }else{
        throw std::runtime_error("Dont understand the aggregation function.");
    }

    if (rank_aggrFunc == "xgboost" && !xgb_booster){
        throw std::runtime_error("XGBoost model is not loaded. Call load_xgboost_model() before ranking.");
    }



    if (verbose && dirIsTail){
        std::cout<<"Calculating tail queries.."<<std::endl;
    }else if (verbose){
        std::cout<<"Calculating head queries.."<<std::endl;
        
    }

    
    int numNodes = train.getIndex()->getNodeSize();
    int numRel = train.getIndex()->getRelSize();
    // size is num triples not queries 
    int chunk = std::min(10000, std::max(1000, (target.getSize())/50));


    std::vector<std::tuple<int,int,int>> tasks;

    for (int rel=0; rel<numRel; rel++){
        for (int source=0; source<numNodes; source++){
                int* begin;
                int length;
                dirIsTail ? target.getTforHR(source, rel, begin, length) : target.getHforTR(source, rel, begin, length);
                if (length>0){
                    tasks.emplace_back(rel, source, length);
                }
        }
    }
    int ctr=0;
    std::atomic<bool> errorFlag(false);
    std::string errorMsg;
    #pragma omp parallel num_threads(num_thr)
    {
        QueryResults qResults(rank_topk, rank_discAtLeast);
        qResults.setPerformAggregation(performAggregation);
        qResults.setAggrFunc(rank_aggrFunc);
        qResults.setNumTopRules(score_numTopRules);
        ManySet filter;
        #pragma omp for schedule(dynamic)
        for (int i=0; i<tasks.size(); i++){
            if (errorFlag.load()){
                continue;
            }
            try {
                int rel = std::get<0>(tasks[i]);
                int source = std::get<1>(tasks[i]);
                int length = std::get<2>(tasks[i]);

            int adapted_topk = rank_topk;   
            if (adapt_topk){
                adapted_topk = rank_topk + length;
                qResults.setAddTopK(adapted_topk);
            }
            ctr+=1;
            if (verbose && ctr%chunk==0 && dirIsTail){
                std::cout<<"Calculated "<< (ctr/chunk) * chunk <<" tail queries..."<<std::endl;
            }else if (verbose && ctr%chunk==0){
                std::cout<<"Calculated "<< (ctr/chunk) * chunk <<" head queries..."<<std::endl;
            }
            auto& relRules = rules.getRelRules(rel);
             // filtering for train and additionalFilter
            if (rank_filterWtrain){
                Nodes* trainFilter = nullptr;
                trainFilter = (!dirIsTail) ? train.getHforTR(source, rel) : train.getTforHR(source, rel);
                if (trainFilter){
                    filter.addSet(trainFilter);
                }   
                        
            }
            // always filter with additionalFilter (can be empty)
            Nodes* naddFilter = nullptr;
            naddFilter = (!dirIsTail) ? addFilter.getHforTR(source, rel) : addFilter.getTforHR(source, rel);
            if (naddFilter){
                filter.addSet(naddFilter);
            }
            // perform rule application
            int ctr = 0;
            int currSize = 0;
            for (Rule* rule : relRules){
                ctr += 1;
                (rule->*predictHeadOrTail)(source, train, qResults, filter);
                currSize = qResults.size();
                if (rank_numPreselect>0 && currSize>=rank_numPreselect){
                    break;
                }
                // possibly can be optimized
                // checking for discrimination after every rule had no noticeable overhead
                if (currSize>=adapted_topk){
                    if (rank_discAtLeast>0){
                         if (qResults.checkDiscrimination()){
                            break;
                         }
                    }
                    if (score_numTopRules>0){
                        if (qResults.checkNumTopRules()){
                             break;
                        }
                    }
                 }
            }

            std::vector<std::pair<int, double>> sortedCandScores;
            // tie handling, final processing, sorting
                if (performAggregation){
                    (this->*sortAndProcess)(sortedCandScores, qResults, train);
                }

            if (rank_aggrFunc=="noisyor" && omp_get_thread_num()==0){
                int* gtBegin = nullptr;
                int gtLength = 0;
                dirIsTail ? target.getTforHR(source, rel, gtBegin, gtLength) : target.getHforTR(source, rel, gtBegin, gtLength);
                std::unordered_set<int> gtSet;
                gtSet.reserve(gtLength);
                for (int gi=0; gi<gtLength; gi++){
                    gtSet.insert(gtBegin[gi]);
                }

                std::unordered_set<int> allRuleIds;
                std::vector<int> candidateOrder = qResults.getCandsOrdered();
                double globalMaxConf = -1.0;
                for (int cand : candidateOrder){
                    auto& rulesForCand = qResults.getRulesForCand(cand);
                    std::unordered_set<int> seen;
                    for (Rule* rule : rulesForCand){
                        int rid = rule->getID();
                        if (seen.insert(rid).second){
                            allRuleIds.insert(rid);
                            double conf = rule->getConfidence();
                            if (conf > globalMaxConf){
                                globalMaxConf = conf;
                            }
                        }
                    }
                }
                std::vector<int> allRuleIdsVec(allRuleIds.begin(), allRuleIds.end());
                std::sort(allRuleIdsVec.begin(), allRuleIdsVec.end());

                Index* index = target.getIndex();
                std::ostringstream oss;
                oss << "{\"query\":\"";
                std::string relStr = index->getStringOfRelId(rel);
                if (dirIsTail){
                    std::string srcStr = index->getStringOfNodeId(source);
                    oss << srcStr << " " << relStr << " ?";
                }else{
                    std::string srcStr = index->getStringOfNodeId(source);
                    oss << "? " << relStr << " " << srcStr;
                }
                oss << "\",\"rules\":[";
                for (size_t i=0; i<allRuleIdsVec.size(); i++){
                    if (i>0) oss << ",";
                    oss << allRuleIdsVec[i];
                }
                oss << "],\"candidates\":[";

                bool firstCand = true;
                bool hasGtCandidate = false;
                std::vector<int> filteredCandidates;
                filteredCandidates.reserve(candidateOrder.size());
                for (int cand : candidateOrder){
                    auto& rulesForCand = qResults.getRulesForCand(cand);
                    std::unordered_set<int> seen;
                    int uniqueRules = 0;
                    double candMaxConf = -1.0;
                    for (Rule* rule : rulesForCand){
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
                    auto& rulesForCand = qResults.getRulesForCand(cand);
                    std::vector<int> ruleIds;
                    std::vector<Rule*> appliedRules;
                    std::unordered_map<int, Rule*> appliedById;
                    ruleIds.reserve(rulesForCand.size());
                    appliedRules.reserve(rulesForCand.size());
                    for (Rule* rule : rulesForCand){
                        int rid = rule->getID();
                        if (appliedById.find(rid) == appliedById.end()){
                            appliedById[rid] = rule;
                            ruleIds.push_back(rid);
                            appliedRules.push_back(rule);
                        }
                    }

                    std::unordered_map<Rule*, double> ruleSurprisal;
                    ruleSurprisal.reserve(appliedRules.size());
                    double originalSurprisal = 0.0;
                    for (Rule* rule : appliedRules){
                        double s = -std::log(1 - rule->getConfidence());
                        ruleSurprisal[rule] = s;
                        originalSurprisal += s;
                    }

                    int positiveDep = 0;
                    int negativeDep = 0;
                    bool usePositive = (rank_dependencyMethod == "positive" || rank_dependencyMethod == "all");
                    bool useNegative = (rank_dependencyMethod == "negative" || rank_dependencyMethod == "all");
                    std::unordered_map<Rule*, double> outPosSum;
                    std::unordered_map<Rule*, int> outPosDeg;
                    std::unordered_map<Rule*, double> inNegSum;
                    std::unordered_map<Rule*, int> inNegDeg;
                    if (usePositive || useNegative){
                        for (Rule* rule : appliedRules){
                            int rid = rule->getID();
                            for (const auto& depPair : rule->dependency){
                                int otherId = depPair.first;
                                Dependency* dep = depPair.second;
                                if (!dep || dep->i != rid){
                                    continue;
                                }
                                auto itOther = appliedById.find(otherId);
                                if (itOther == appliedById.end()){
                                    continue;
                                }
                                if (dep->lift > 0){
                                    if (usePositive){
                                        positiveDep += 1;
                                        outPosSum[rule] += std::abs(dep->lift);
                                        outPosDeg[rule] += 1;
                                    }
                                }else if (dep->lift < 0){
                                    if (useNegative){
                                        negativeDep += 1;
                                        inNegSum[itOther->second] += std::abs(dep->lift);
                                        inNegDeg[itOther->second] += 1;
                                    }
                                }
                            }
                        }
                    }

                    std::vector<double> newRuleSurprisal;
                    std::vector<double> ruleSurprisalList;
                    newRuleSurprisal.reserve(appliedRules.size());
                    ruleSurprisalList.reserve(appliedRules.size());
                    for (Rule* rule : appliedRules){
                        double base = -std::log(1 - rule->getConfidence());
                        ruleSurprisalList.push_back(base);
                        double outVal = 0.0;
                        double inVal = 0.0;
                        auto itOutDeg = outPosDeg.find(rule);
                        if (itOutDeg != outPosDeg.end() && itOutDeg->second > 0){
                            outVal = outPosSum[rule] / static_cast<double>(itOutDeg->second);
                        }
                        auto itInDeg = inNegDeg.find(rule);
                        if (itInDeg != inNegDeg.end() && itInDeg->second > 0){
                            inVal = inNegSum[rule] / static_cast<double>(itInDeg->second);
                        }
                        double adjusted = base + rank_positiveWeight * outVal - rank_negativeWeight * inVal;
                        adjusted = std::min(7.0, std::max(0.0, adjusted));
                        newRuleSurprisal.push_back(adjusted);
                    }

                    std::vector<double> surprisals;
                    surprisals.reserve(appliedRules.size());
                    for (double s : newRuleSurprisal){
                        if (s > 0){
                            surprisals.push_back(s);
                        }
                    }
                    std::sort(surprisals.begin(), surprisals.end(), std::greater<double>());
                    double newSurprisal = 0.0;
                    double tau = rank_aggrSharpness;
                    for (size_t i=0; i<surprisals.size(); i++){
                        newSurprisal += surprisals[i] * std::exp(-tau * static_cast<double>(i));
                    }

                    if (!firstCand) oss << ",";
                    firstCand = false;
                    std::string candStr = index->getStringOfNodeId(cand);
                    oss << "{\"name\":\"" << candStr << "\",\"rules\":[";
                    for (size_t i=0; i<ruleIds.size(); i++){
                        if (i>0) oss << ",";
                        oss << ruleIds[i];
                    }
                    oss << "],\"ruleSurprisal\":[";
                    for (size_t i=0; i<ruleSurprisalList.size(); i++){
                        if (i>0) oss << ",";
                        oss << ruleSurprisalList[i];
                    }
                    oss << "],\"newRuleSurprisal\":[";
                    for (size_t i=0; i<newRuleSurprisal.size(); i++){
                        if (i>0) oss << ",";
                        oss << newRuleSurprisal[i];
                    }
                    oss << "],\"GT\":" << (gtSet.find(cand)!=gtSet.end() ? "true" : "false");
                    oss << ",\"positiveDep\":" << positiveDep;
                    oss << ",\"negativeDep\":" << negativeDep;
                    oss << ",\"originalSurprisal\":" << originalSurprisal;
                    oss << ",\"newSurprisal\":" << newSurprisal;
                    oss << "}";
                }

                oss << "]}";
                std::cout << oss.str() << std::endl;
            }
                    

            #pragma omp critical
            {   
                if (saveCandidateRules){
                    // TODO when needed could prevent copy here by using shared pointer
                    if (dirIsTail){
                        tailQcandsRules[rel][source] = qResults.getCandRules();
                    }else{
                        headQcandsRules[rel][source] = qResults.getCandRules();
                    }
                }
                if (performAggregation){
                        auto& writeResults = (dirIsTail) ? tailQcandsConfs : headQcandsConfs;
                        writeResults[rel][source] = sortedCandScores;
                }
            }
                qResults.clear();
                filter.clear();
            } catch (const std::exception& e){
                bool expected = false;
                if (errorFlag.compare_exchange_strong(expected, true)){
                    errorMsg = e.what();
                }
                qResults.clear();
                filter.clear();
                continue;
            } catch (...){
                bool expected = false;
                if (errorFlag.compare_exchange_strong(expected, true)){
                    errorMsg = "Unknown error";
                }
                qResults.clear();
                filter.clear();
                continue;
            }
        } 
    } //pragma

    if (errorFlag.load()){
        throw std::runtime_error("Ranking failed: " + errorMsg);
    }
}

void ApplicationHandler::sortAndProcessNoisy(std::vector<std::pair<int,double>>& candScoresToSort, QueryResults& qResults, TripleStorage& data){
    // compute noisyor score from all applied rules: sort surprisal desc and apply exp decay
    // w0 + w1 * e^-τ + w2 * e^-2τ ...
    NodeToPredRules& candRules = qResults.getCandRules();
    candScoresToSort.clear();
    candScoresToSort.reserve(candRules.size());
    for (auto& candPair : candRules){
        std::unordered_map<int, Rule*> appliedById;
        std::vector<Rule*> appliedRules;
        appliedRules.reserve(candPair.second.size());
        for (Rule* rule : candPair.second){
            int rid = rule->getID();
            if (appliedById.find(rid) == appliedById.end()){
                appliedById[rid] = rule;
                appliedRules.push_back(rule);
            }
        }

        std::unordered_map<Rule*, double> ruleSurprisal;
        ruleSurprisal.reserve(appliedRules.size());
        for (Rule* rule : appliedRules){
            ruleSurprisal[rule] = -std::log(1 - rule->getConfidence());
        }

        bool usePositive = (rank_dependencyMethod == "positive" || rank_dependencyMethod == "all");
        bool useNegative = (rank_dependencyMethod == "negative" || rank_dependencyMethod == "all");
        std::unordered_map<Rule*, double> outPosSum;
        std::unordered_map<Rule*, int> outPosDeg;
        std::unordered_map<Rule*, double> inNegSum;
        std::unordered_map<Rule*, int> inNegDeg;
        if (usePositive || useNegative){
            for (Rule* rule : appliedRules){
                int rid = rule->getID();
                for (const auto& depPair : rule->dependency){
                    int otherId = depPair.first;
                    Dependency* dep = depPair.second;
                    if (!dep){
                        continue;
                    }
                    if (dep->i != rid){
                        continue;
                    }
                    auto itOther = appliedById.find(otherId);
                    if (itOther == appliedById.end()){
                        continue;
                    }
                    if (dep->lift > 0){
                        if (usePositive){
                            outPosSum[rule] += std::abs(dep->lift);
                            outPosDeg[rule] += 1;
                        }
                    }else if (dep->lift < 0){
                        if (useNegative){
                            inNegSum[itOther->second] += std::abs(dep->lift);
                            inNegDeg[itOther->second] += 1;
                        }
                    }
                }
            }
        }

        std::vector<double> surprisals;
        surprisals.reserve(appliedRules.size());
        for (Rule* rule : appliedRules){
            double outVal = 0.0;
            double inVal = 0.0;
            auto itOutDeg = outPosDeg.find(rule);
            if (itOutDeg != outPosDeg.end() && itOutDeg->second > 0){
                outVal = outPosSum[rule] / static_cast<double>(itOutDeg->second);
            }
            auto itInDeg = inNegDeg.find(rule);
            if (itInDeg != inNegDeg.end() && itInDeg->second > 0){
                inVal = inNegSum[rule] / static_cast<double>(itInDeg->second);
            }
            double adjusted = ruleSurprisal[rule] + rank_positiveWeight * outVal - rank_negativeWeight * inVal;
            adjusted = std::min(7.0, std::max(0.0, adjusted));
            if (adjusted > 0){
                surprisals.push_back(adjusted);
            }
        }

        std::sort(surprisals.begin(), surprisals.end(), std::greater<double>());
        double surprisal = 0.0;
        double tau = rank_aggrSharpness;
        for (size_t i=0; i<surprisals.size(); i++){
            surprisal += surprisals[i] * std::exp(-tau * static_cast<double>(i));
        }
        candScoresToSort.emplace_back(candPair.first, surprisal);
    }

   if (rank_tie_handling=="random"){
     std::sort(
        candScoresToSort.begin(),
        candScoresToSort.end(), 
        [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
            return a.second > b.second;
        }
     );
   }else if (rank_tie_handling=="frequency"){
      std::sort(
        candScoresToSort.begin(),
        candScoresToSort.end(), 
        [&data](const std::pair<int, double>& a, const std::pair<int, double>& b) {
            if (a.second!=b.second){
                return a.second > b.second;
            }else if(data.getFreq(a.first) != data.getFreq(b.first)) {
                return data.getFreq(a.first) > data.getFreq(b.first);
            }else{
                return a.first<b.first;
            }
        }
      );
   }else{
    throw std::runtime_error("Tie handling type not known. Please set to 'random' or 'frequency'");
   }

    for (auto& pair: candScoresToSort){
        pair.second = 1 - std::exp(-1*pair.second);
    }

}

void ApplicationHandler::sortAndProcessMax(std::vector<std::pair<int,double>>& candScoresToSort, QueryResults& qResults, TripleStorage& data){
    scoreMaxPlus(qResults.getCandRules(), candScoresToSort, data);
}

void ApplicationHandler::sortAndProcessXGBoost(std::vector<std::pair<int,double>>& candScoresToSort, QueryResults& qResults, TripleStorage& data){
    if (!xgb_booster){
        throw std::runtime_error("XGBoost model is not loaded. Call load_xgboost_model() first.");
    }

    NodeToPredRules& candRules = qResults.getCandRules();
    candScoresToSort.clear();
    if (candRules.empty()){
        return;
    }

    const size_t nrows = candRules.size();
    const size_t ncols = 40;
    std::vector<float> matrix;
    matrix.resize(nrows * ncols, 0.0f);
    std::vector<int> candIds;
    candIds.reserve(nrows);

    size_t row = 0;
    for (const auto& candPair : candRules){
        DepGraphFeatures f = extractFeaturesFromRules(candPair.second);
        std::array<float, 40> rowVals;
        fillFeatureRow(f, rowVals);
        for (size_t c = 0; c < ncols; c++){
            matrix[row * ncols + c] = rowVals[c];
        }
        candIds.push_back(candPair.first);
        row += 1;
    }

    DMatrixHandle dmat = nullptr;
    if (XGDMatrixCreateFromMat(matrix.data(), static_cast<bst_ulong>(nrows), static_cast<bst_ulong>(ncols), std::numeric_limits<float>::quiet_NaN(), &dmat) != 0){
        throw std::runtime_error(XGBGetLastError());
    }

    bst_ulong outLen = 0;
    const float* outResult = nullptr;
    std::vector<float> preds;
    static std::mutex xgbPredictMutex;
    {
        std::lock_guard<std::mutex> lock(xgbPredictMutex);
        if (XGBoosterPredict(static_cast<BoosterHandle>(xgb_booster), dmat, 0, 0, 0, &outLen, &outResult) != 0){
            XGDMatrixFree(dmat);
            throw std::runtime_error(XGBGetLastError());
        }
        if (outResult && outLen > 0){
            preds.assign(outResult, outResult + outLen);
        }
    }
    XGDMatrixFree(dmat);

    candScoresToSort.reserve(nrows);
    for (size_t i = 0; i < nrows; i++){
        double score = (i < preds.size()) ? static_cast<double>(preds[i]) : 0.0;
        candScoresToSort.emplace_back(candIds[i], score);
    }

    if (rank_tie_handling=="random"){
        std::sort(
            candScoresToSort.begin(),
            candScoresToSort.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second > b.second;
            }
        );
    } else if (rank_tie_handling=="frequency"){
        std::sort(
            candScoresToSort.begin(),
            candScoresToSort.end(),
            [&data](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                if (a.second!=b.second){
                    return a.second > b.second;
                } else if(data.getFreq(a.first) != data.getFreq(b.first)) {
                    return data.getFreq(a.first) > data.getFreq(b.first);
                } else {
                    return a.first < b.first;
                }
            }
        );
    } else {
        throw std::runtime_error("Tie handling type not known. Please set to 'random' or 'frequency'");
    }
}

// currently not used in the ranking process
void ApplicationHandler::aggregateQueryResults(std::string direction, TripleStorage& train){
    auto& queryResults = (direction=="tail") ? tailQcandsRules : headQcandsRules;
    auto& writeResults = (direction=="tail") ? tailQcandsConfs : headQcandsConfs;
    for (auto& queries: queryResults){
            int relation = queries.first;
            std::unordered_map<int, NodeToPredRules>& srcToCand = queries.second;
            for (auto& query: srcToCand){
                int source = query.first; 
                if (rank_aggrFunc=="maxplus"){
                    scoreMaxPlus(query.second, writeResults[relation][source], train);
                }else{
                    throw std::runtime_error("Aggregation function is not recognized in calculate ranking.");
                }
                
            }
    }
}

// note this does not yet filter with target as ranking is performed query based; filtering with target only happens when writing the ranking
void ApplicationHandler::makeRanking(TripleStorage& target, TripleStorage& train, RuleStorage& rules, TripleStorage& addFilter){
    if (rank_tie_handling=="frequency"){
        if (verbose){
            std::cout<<"Calculate entity frequencies..."<<std::endl;
        }
        
        train.calcEntityFreq();
    }
    calculateQueryResults(target, train, rules, addFilter, true);
    calculateQueryResults(target, train, rules, addFilter, false);
}

// query results must have been calculated before and aggregated
void ApplicationHandler::writeRanking(TripleStorage& target, std::string filepath){
    Index* index = target.getIndex();
    RelNodeToNodes& data = target.getRelTailToHeads();
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw  std::runtime_error("Failed to create file. Please check if the paths are correct: " + filepath );
    }
    for (auto& relQueries: data){
        int relation = relQueries.first;
        for (auto& srcTocands: relQueries.second){
            int tail = srcTocands.first;
            // true heads
            Nodes& trueHeads = srcTocands.second;
            // we use this direction to iterate over all triples
            // head relation tail is one triple of the target set
            for (int head: trueHeads){
                if (file.is_open()){
                    file<<index->getStringOfNodeId(head)<<" "<<index->getStringOfRelId(relation)<<" "<<index->getStringOfNodeId(tail)<<std::endl;
                    file<<"Heads: ";
                    // write head ranking
                    CandidateConfs& resultsHead = headQcandsConfs[relation][tail];
                    int numWritten = 0;
                    for (int i=0; i<resultsHead.size(); i++){
                        auto pair = resultsHead[i];
                        int predHead = pair.first;
                        double score = pair.second;
                        // filter with target
                        // current predicted head is excluded if its the true answer to some other query
                        if (rank_filterWtarget && !(predHead==head)){
                            if (!(trueHeads.find(predHead)==trueHeads.end())){
                                continue;
                            }
                        }
                        file<<index->getStringOfNodeId(predHead)<<"\t"<<score<<"\t";
                        numWritten += 1;
                        if (numWritten==rank_topk){
                            break;
                        }
                    }
                    // write tail ranking
                    file<<"\nTails: ";
                    //true tails for filtering
                    Nodes& trueTails = target.getRelHeadToTails()[relation][head];
                    CandidateConfs& resultsTail = tailQcandsConfs[relation][head];
                    numWritten = 0;
                    for (int i=0; i<resultsTail.size(); i++){
                        auto pair = resultsTail[i];
                        int predTail = pair.first;
                        double score = pair.second;
                        if (rank_filterWtarget && !(predTail==tail)){
                            if (!(trueTails.find(predTail)==trueTails.end())){
                                continue;
                            }
                        }
                        file<<index->getStringOfNodeId(predTail)<<"\t"<<score<<"\t";
                        numWritten += 1;
                        if (numWritten==rank_topk){
                            break;
                        }
                    }
                    file<<"\n";
                }
            }
        }
    }
    file.close();
    std::cout<<"Ranking file written to:  " + filepath <<std::endl; 
}

// query results must have been calculated before and aggregated
void ApplicationHandler::writeRules(TripleStorage& target, std::string filepath, std::string direction, bool strings){
    
    if ((this->headQcandsRules.size() == 0 && this->tailQcandsRules.size() == 0) || !saveCandidateRules){
        throw std::runtime_error(
            "Please calculate answers using calculate_ranking() and set in the options ranking_handler.collect_rules to true first."
        );
    }

    Index* index = target.getIndex();
    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw  std::runtime_error("Failed to create file. Please check if the paths are correct: " + filepath );
    }

    auto& data = (direction == "head") ? this->headQcandsRules : this->tailQcandsRules;

    for (auto& relQueries: data){
        int relation = relQueries.first;
        std::string relationStr = strings ? "\"" + index->getStringOfRelId(relation) + "\"" : std::to_string(relation);
        for (auto& srcQueries: relQueries.second){
            int src = srcQueries.first;
            std::string srcStr = strings ? "\"" + index->getStringOfNodeId(src) + "\"" : std::to_string(src);
        
            // Collect answers and rules
            std::string answers = "";
            std::string rules = "";

            auto itr = srcQueries.second.begin();
            for(; itr != srcQueries.second.end(); itr++){
                int to = itr->first;
                std::string toStr = strings ? "\"" + index->getStringOfNodeId(to) + "\"" : std::to_string(to);
                answers += toStr;
                if (std::next(itr) != srcQueries.second.end()) {
                    answers += ",";
                }

                std::string ruleset = "[";
                for(int ridx = 0; ridx < itr->second.size(); ridx++){
                    ruleset += strings ? "\"" + itr->second[ridx]->computeRuleString(index) + "\"" : std::to_string(itr->second[ridx]->getID());
                    if (ridx < itr->second.size() - 1){
                        ruleset += ",";
                    }
                }
                ruleset += "]";
                rules += ruleset;
                if (std::next(itr) != srcQueries.second.end()) {
                    rules += ",";
                }
            }
            file << "{\"query\": [" << srcStr << "," << relationStr << "], \"answers\": [" << answers << "], \"rules\": [" << rules << "]}" << std::endl;
        }
    }
    file.close();
    std::cout<<"Rules file written to:  " + filepath <<std::endl; 
}

void ApplicationHandler::scoreMaxPlus(
    const NodeToPredRules& candToRules, std::vector<std::pair<int, double>>& aggrCand, TripleStorage& train
     ){
    
    // for noisy-or we can simply sort according to aggrCand after scoring
    // here we have to sort and score separately
    std::vector<std::pair<int, std::vector<Rule*>>> candsToSort(candToRules.begin(), candToRules.end());

    // max+ sorting
    auto sortLexicographic = [&train, this](const std::pair<int, std::vector<Rule*>>& candA, const std::pair<int, std::vector<Rule*>>& candB) { 
        std::vector<Rule*> rulesA = candA.second;
        std::vector<Rule*> rulesB = candB.second;

        int minRules = std::min(rulesA.size(), rulesB.size());
        for (int i=0; i<minRules; i++){
            double confA = rulesA[i]->getConfidence();
            double confB = rulesB[i]->getConfidence();
            if (confA > confB){
                return true;
            } else if (confB > confA){
                return false;
            }
        }
        // all compared rules were equal rank according to num rules
        if (rulesB.size() > rulesA.size()){
            return false;
        } else if (rulesA.size() > rulesB.size()){
            return true;
        }
        //exactly the same rules given NodeToPred is unordered_map return random order
        if (this->rank_tie_handling=="random"){
            return false;
        }else if (this->rank_tie_handling=="frequency"){
            if (train.getFreq(candA.first)!=train.getFreq(candB.first)){
                return train.getFreq(candA.first) > train.getFreq(candB.first);
            }else{
                return candA.first<candB.first;
            }
            
        } else {
            throw std::runtime_error("Could not understand tie_handling_paramter in scoreMaxPlus.");
        }
    };
     std::sort(candsToSort.begin(), candsToSort.end(), sortLexicographic);
    

    // take sorted candidate and derive its score according to highest rule
     for (auto& pair: candsToSort){
        aggrCand.push_back(
            std::make_pair(
                pair.first,
                pair.second[0]->getConfidence()
                )
            );
     }
}

void ApplicationHandler::clearAll(){
    headQcandsRules.clear();
    headQcandsConfs.clear();
    tailQcandsRules.clear();
    tailQcandsConfs.clear();
    tripleScores.clear();
    tripleGroundings.clear();
}
 
void ApplicationHandler::setNumPreselect(int num){
    rank_numPreselect=num;
}

void ApplicationHandler::setTopK(int topk){
    rank_topk=topk;
}
void ApplicationHandler::setFilterWTrain(bool ind){
    rank_filterWtrain=ind;
}

void ApplicationHandler::setFilterWtarget(bool ind){
    rank_filterWtarget = ind;
}
void ApplicationHandler::setAggregationFunc(std::string func){
    if (!(func=="maxplus") && !(func=="noisyor") && !(func=="xgboost")){
        throw std::runtime_error("The aggregation function value is not known, select from 'noisyor', 'maxplus', or 'xgboost' found value: " + func);
    }
    rank_aggrFunc = func;
}

void ApplicationHandler::setAggregationSharpness(double val){
    rank_aggrSharpness = val;
}

void ApplicationHandler::setDependencyMethod(std::string method){
    rank_dependencyMethod = method;
}

void ApplicationHandler::loadXGBoostModel(std::string path){
    if (xgb_booster && xgb_owns){
        XGBoosterFree(static_cast<BoosterHandle>(xgb_booster));
    }
    BoosterHandle booster;
    if (XGBoosterCreate(nullptr, 0, &booster) != 0){
        throw std::runtime_error(XGBGetLastError());
    }
    if (XGBoosterLoadModel(booster, path.c_str()) != 0){
        XGBoosterFree(booster);
        throw std::runtime_error(XGBGetLastError());
    }
    xgb_booster = booster;
    xgb_owns = true;
}

void ApplicationHandler::setXGBoostModelHandle(void* handle){
    if (xgb_booster && xgb_owns){
        XGBoosterFree(static_cast<BoosterHandle>(xgb_booster));
    }
    xgb_booster = handle;
    xgb_owns = false;
}

bool ApplicationHandler::hasXGBoostModel(){
    return xgb_booster != nullptr;
}

ApplicationHandler::~ApplicationHandler(){
    if (xgb_booster && xgb_owns){
        XGBoosterFree(static_cast<BoosterHandle>(xgb_booster));
        xgb_booster = nullptr;
    }
}

void ApplicationHandler::setPositiveWeight(double val){
    rank_positiveWeight = val;
}

void ApplicationHandler::setNegativeWeight(double val){
    rank_negativeWeight = val;
}


void ApplicationHandler::setSaveCandidateRules(bool ind){
    saveCandidateRules = ind;
}
void  ApplicationHandler::setPerformAggregation(bool ind){
    performAggregation = ind;
}

void ApplicationHandler::setDiscAtLeast(int num){
    rank_discAtLeast = num;
}

void ApplicationHandler::setTieHandling(std::string opt){
    rank_tie_handling = opt;
}

void ApplicationHandler::setVerbose(bool ind){
    verbose = ind;
}

void ApplicationHandler::setScoreCollectGroundings(bool ind){
    score_collectGr = ind;
}

bool ApplicationHandler::getScoreCollectGroundings(){
    return score_collectGr;
}

void ApplicationHandler::setScoreNumTopRules(int num){
    score_numTopRules = num; 
}

std::unordered_map<int,std::unordered_map<int, NodeToPredRules>>& ApplicationHandler::getHeadQcandsRules(){
    return headQcandsRules;
}
std::unordered_map<int,std::unordered_map<int, CandidateConfs>>&  ApplicationHandler::getHeadQcandsConfs(){
    return headQcandsConfs;
}

std::unordered_map<int,std::unordered_map<int, NodeToPredRules>>&  ApplicationHandler::getTailQcandsRules(){
    return tailQcandsRules;
}
std::unordered_map<int,std::unordered_map<int, CandidateConfs>>& ApplicationHandler::getTailQcandsConfs(){
    return tailQcandsConfs;
}

std::vector<std::array<double, 4>>& ApplicationHandler::getTripleScores(){
    return tripleScores;
}
std::vector<std::pair<Triple, RuleGroundings>>& ApplicationHandler::getTripleGroundings(){
    return tripleGroundings;
}

void ApplicationHandler::setNumThr(int num){
    if (num==-1){
        num_thr = omp_get_max_threads();
    }else{
        num_thr = num;
    }
}

void ApplicationHandler::setAdaptTopK(bool ind){
    adapt_topk = ind;
}