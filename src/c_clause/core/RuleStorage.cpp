#include "RuleStorage.h"
#include "Index.h"
#include "Globals.h"
#include "Rule.h"
#include "Types.h"
#include "Util.hpp"

#include <fstream>
#include <string>
#include <utility>
#include <map>


RuleStorage::RuleStorage(std::shared_ptr<Index> index, std::shared_ptr<RuleFactory> ruleFactory){
    this->ruleFactory = ruleFactory;
    this->index = index;
}

namespace {
void printRuleTypeStats(const std::vector<std::unique_ptr<Rule>>& rules){
    if (rules.empty()){
        std::cout << "Rule types: none." << std::endl;
        return;
    }
    std::map<std::string, int> counts;
    for (const auto& rule : rules){
        if (!rule){
            continue;
        }
        const char* type = rule->type;
        std::string key = type ? std::string(type) : std::string("unknown");
        counts[key] += 1;
    }
    std::cout << "Rule types:";
    for (const auto& kv : counts){
        std::cout << " " << kv.first << "=" << kv.second;
    }
    std::cout << std::endl;
}
}


// reads format outputted by AnyBURL
void RuleStorage::readAnyTimeFormat(std::string path, bool exact){
    lineToRulePtrs.clear();
    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::ios_base::failure("Could not open rule file: " + path + " is the path correct?");
    }

    if (verbose){
        std::cout << "Loading rules from " + path << std::endl;
    }

    std::ios_base::sync_with_stdio(false); 
    
    constexpr size_t bufferSize =  256 * 1024; 
    char buffer[bufferSize];
    file.rdbuf()->pubsetbuf(buffer, bufferSize);

    std::string line;
    int currID = 0;
    int currLine = 1;
    
    while (!util::safeGetline(file, line).eof()){
        if (currLine % 1000000 == 0 && verbose && currLine > 0){
            std::cout << "...parsed " << currLine << " rules " << std::endl;
        }
        bool added = addAnyTimeRuleLine(line, currLine, false);
        if (added){
            currID += 1;
        }
        currLine += 1;
    }
    std::cout << "Loaded " << currID << " rules." << std::endl;
    printRuleTypeStats(rules);
}

void RuleStorage::readAnyTimeParFormat(std::string path, bool exact, int numThreads){
     std::ifstream file(path);
    if (!file.is_open()) {
        throw std::ios_base::failure("Could not open rule file: " + path + " is the path correct?");
    }

    if (verbose){
        std::cout << "Loading rules from " + path << std::endl;
    }

    std::ios_base::sync_with_stdio(false); 
    
    constexpr size_t bufferSize =  256 * 1024; 
    char buffer[bufferSize];
    file.rdbuf()->pubsetbuf(buffer, bufferSize);

    std::string line;
    int currLine = 0;

    std::vector<std::string> ruleLines;
    std::vector<std::unique_ptr<Rule>> rules_ptr;


    while (!util::safeGetline(file, line).eof()){
        ruleLines.push_back(line);
    }
    file.close();

    // we not need all of them 
    rules_ptr.resize(ruleLines.size());
    // 1-based index: line number -> Rule*
    lineToRulePtrs.clear();
    lineToRulePtrs.resize(ruleLines.size() + 1, nullptr);

    #pragma omp parallel num_threads(numThreads)
    {
        #pragma omp for //no need for dynamic
        for (int i=0; i<ruleLines.size(); i++){

            if (i>0 && i%1000000==0){
                std::cout<<"parsed 1 million rules..." <<std::endl;
            }

            std::string ruleLine = ruleLines[i];

            // expects a line: predicted\t cpredicted\tconf\trulestring
            std::vector<std::string> splitline = util::split(ruleLine, '\t');

            if (splitline.size()==1){
                int numPreds = 100;
                int numTrue = 100;
                std::cout<<"Warning: could not find num preds and support for input line " + splitline[0]<<std::endl;
                std::cout<<" Setting both to 100. Expect random ordering for rules and predictions, confidence scores will all be 1."<<std::endl;
            }

            if (splitline.size()!=4){
                std::cout<<"Could not parse this rule because of line format: " + ruleLine<<std::endl;
                std::cout<<"Skipping but please check your format."<<std::endl;

            }
            std::string ruleString = splitline[3];
            int numPreds = std::stoi(splitline[0]);
            int numTrue = std::stoi(splitline[1]);
            std::unique_ptr<Rule> rule = nullptr;
            try
                {
                    rule = ruleFactory->parseAnytimeRule(ruleString, numPreds, numTrue);
                }
            catch(const std::exception& e)
                {
                    std::cout<<"Could not parse this rule " + ruleLine<<std::endl;
                    std::cout<<"Skipping it, but please check your format."<<std::endl;
                    std::cout<<"And check that if entities are contained, that they are loaded with the data."<<std::endl;
                }

            if (rule){
                rule->setStats(numPreds, numTrue, exact);
                rules_ptr.at(i) = std::move(rule);              
            }
        }
    }
    // need this for correctly setting ID's
    // e.g. we want ID's to be the line number in the input file (1-based)
    // to be consistent when rules are written and loaded again
    std::cout<< "Indexing rules.." <<std::endl;
    int currID = 0;
    for (int i=0; i<rules_ptr.size(); i++){
        if (rules_ptr[i]){
            int lineNumber = i + 1;
            rules_ptr[i]->setID(lineNumber);
            // must be done after id is set
            relToRules[rules_ptr[i]->getTargetRel()].insert(rules_ptr[i].get());
            currID += 1;
            rules.push_back(std::move(rules_ptr[i]));
            lineToRulePtrs[lineNumber] = rules.back().get();
        }
    }
    std::cout<<"Loaded and indexed "<<currID<<" rules."<<std::endl;
    printRuleTypeStats(rules);
}

// ruleStrings is a line num_pred/t support/t conf/t ruleString
void RuleStorage::readAnyTimeFromVec(std::vector<std::string>& ruleStrings, bool exact){
    lineToRulePtrs.clear();
    int currID = 0;
    for (int i=0; i<ruleStrings.size(); i++){
         if (i%1000000==0 && verbose && i>0){
                std::cout<<"...serialized "<<i<<" rules "<<std::endl;
        }
        std::string stringLine = ruleStrings[i];
        bool added = addAnyTimeRuleLine(stringLine, i + 1, exact);
        if (added){
            currID += 1;
        }
    }
    std::cout<<"Loaded "<<currID<<" rules."<<std::endl;
    printRuleTypeStats(rules);
} 

void RuleStorage::readAnyTimeFromVecs(std::vector<std::string>& ruleStrings, std::vector<std::pair<int,int>> stats, bool exact){
    lineToRulePtrs.clear();
    if (ruleStrings.size() != stats.size()){
        throw std::runtime_error(
            "The rule stats input list must have same length of rule string list when loading rules with stats."
        );
    }
    int currID = 0;
    for (int i=0; i<ruleStrings.size(); i++){
        if (i%1000000==0 && verbose && i>0){
                std::cout<<"...serialized "<<i<<" rules "<<std::endl;
        }
        std::string stringRule = ruleStrings[i];
        int numPred = stats[i].first;
        int numTrue = stats[i].second;
        bool added = addAnyTimeRuleWithStats(stringRule, i + 1, numPred, numTrue, exact);
        if (added){
            currID += 1;
        }
    }
    std::cout<<"Loaded "<<currID<<" rules."<<std::endl;
    printRuleTypeStats(rules);

}

bool RuleStorage::addAnyTimeRuleLine(std::string ruleLine, int id , bool exact){
    // expects a line: predicted\t cpredicted\tconf\trulestring
	std::vector<std::string> splitline = util::split(ruleLine, '\t');

    if (splitline.size()==1){
        int numPreds = 100;
        int numTrue = 100;
        std::cout<<"Warning: could not find num preds and support for input line " + splitline[0]<<std::endl;
        std::cout<<" Setting both to 100. Expect random ordering for rules and predictions, confidence scores will all be 1."<<std::endl;
        return addAnyTimeRuleWithStats(splitline[0], id, numPreds, numTrue, exact);
    }

    if (splitline.size()!=4){
        throw std::runtime_error("Could not parse this rule because of format: " + ruleLine);
    }
    std::string ruleString = splitline[3];
    int numPreds = std::stoi(splitline[0]);
    int numTrue = std::stoi(splitline[1]);

    return addAnyTimeRuleWithStats(ruleString, id, numPreds, numTrue, exact);
}

bool RuleStorage::addAnyTimeRuleWithStats(std::string ruleString, int id, int numPred, int numTrue, bool exact){
    std::unique_ptr<Rule> rule = ruleFactory->parseAnytimeRule(ruleString, numPred, numTrue);
    if (rule){
        rule->setID(id);
        rule->setStats(numPred, numTrue, exact);
        relToRules[rule->getTargetRel()].insert(rule.get());
        if (id >= static_cast<int>(lineToRulePtrs.size())){
            lineToRulePtrs.resize(id + 1, nullptr);
        }
        lineToRulePtrs[id] = rule.get();
        rules.push_back(std::move(rule));
        return true;
    } else {
        return false;
    }
}


std::set<Rule*, compareRule>&  RuleStorage::getRelRules(int relation){
    return relToRules[relation];
}

std::unordered_map<int, std::set<Rule*,compareRule>>& RuleStorage::getRelToRules(){
    return relToRules;
}

std::vector<std::unique_ptr<Rule>>& RuleStorage::getRules(){
    return rules;
 }

const std::vector<Rule*>& RuleStorage::getLineToRulePtrs() const{
    return lineToRulePtrs;
}

void RuleStorage::loadDependency(std::string path, int numThreads){
    if (lineToRulePtrs.empty()){
        throw std::runtime_error("Please load rules before loading dependencies.");
    }

    std::ifstream file(path);
    if (!file.is_open()) {
        throw std::ios_base::failure("Could not open dependency file: " + path + " is the path correct?");
    }

    if (verbose){
        std::cout << "Loading dependencies from " + path << std::endl;
    }

    std::vector<std::string> depLines;
    std::string line;
    while (!util::safeGetline(file, line).eof()){
        if (!line.empty()){
            depLines.push_back(line);
        }
    }
    file.close();

    for (Rule* r : lineToRulePtrs){
        if (r){
            r->dependency.clear();
        }
    }
    dependency.clear();
    dependency.reserve(depLines.size());

    std::vector<std::unique_ptr<Dependency>> deps;
    deps.resize(depLines.size());

    int parseError = 0;
    int invalidFormat = 0;
    int outOfRange = 0;
    int missingRule = 0;
    // long long parsedCount = 0;

    if (verbose){
        std::cout << "Starting to parse " << depLines.size() << " dependency lines using " << numThreads << " threads." << std::endl;
    }
    #pragma omp parallel num_threads(numThreads)
    {
        #pragma omp for
        for (int i=0; i<depLines.size(); i++){
            // long long currParsed = 0;
            // #pragma omp atomic capture
            // {
            //     parsedCount += 1;
            //     currParsed = parsedCount;
            // }
            // if (verbose && currParsed % 1000000 == 0){
            //     #pragma omp critical
            //     {
            //         std::cout << "parsed " << currParsed / 1000000 << " million dependencies..." << std::endl;
            //     }
            // }
            std::vector<std::string> splitline = util::split(depLines[i], '\t');
            if (splitline.size()!=8){
                #pragma omp atomic
                invalidFormat += 1;
                continue;
            }
            try {
                int predicted = std::stoi(splitline[0]);
                int cpredicted = std::stoi(splitline[1]);
                double conf = std::stod(splitline[2]);
                double lift = std::stod(splitline[3]);
                double conf1 = std::stod(splitline[4]);
                double conf2 = std::stod(splitline[5]);
                int id1 = std::stoi(splitline[6]);
                int id2 = std::stoi(splitline[7]);

                if (id1 == id2){
                    continue;
                }
                if (id1 > id2){
                    std::swap(id1, id2);
                    std::swap(conf1, conf2);
                }

                if (id1 >= static_cast<int>(lineToRulePtrs.size()) || id2 >= static_cast<int>(lineToRulePtrs.size())){
                    #pragma omp atomic
                    outOfRange += 1;
                    continue;
                }
                Rule* r1 = lineToRulePtrs[id1];
                if (!r1){
                    #pragma omp atomic
                    missingRule += 1;
                    continue;
                }

                auto dep = std::make_unique<Dependency>();
                dep->i = id1;
                dep->j = id2;
                dep->predicted = predicted;
                dep->cpredicted = cpredicted;
                dep->conf = conf;
                dep->lift = lift;
                dep->conf1 = conf1;
                dep->conf2 = conf2;
                deps[i] = std::move(dep);

                #pragma omp critical
                {
                    r1->dependency[id2] = deps[i].get();
                    dependency.push_back(std::move(deps[i]));
                }
            } catch (...) {
                #pragma omp atomic
                parseError += 1;
                continue;
            }
        }
    }

    if (verbose){
        std::cout << "Loaded " << dependency.size() << " dependency";
        int totalSkipped = outOfRange + missingRule;
        if (totalSkipped > 0){
            std::cout << " (skipped " << totalSkipped
                      << ", out_of_range=" << outOfRange
                      << ", missing_rule=" << missingRule
                      << ", invalid_format=" << invalidFormat
                      << ", parse_error=" << parseError
                      << ")";
        }
        std::cout << "." << std::endl;
    }
}

void RuleStorage::clearAll(){
    rules.clear();
    relToRules.clear();
    lineToRulePtrs.clear();
    dependency.clear();
}