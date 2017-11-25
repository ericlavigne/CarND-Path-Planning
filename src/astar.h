#ifndef PATH_PLANNING_ASTAR_H
#define PATH_PLANNING_ASTAR_H

#include <unordered_map>
#include <set>
#include <queue>
#include <string>
#include <algorithm>

using namespace std;

/*
    Performs A* search for "State" class that implements the following methods:
        State(const State &state);
        double scoreEstimate(); // Higher is better. Must be greater than or equal to true value.
        vector<State> nextStates();
        string key(); // Must be unique. Used to determine if we've seen this state before.
        string show();
        bool final(); // True if this state is candidate to be the end of a path.
        bool operator<(const State& rhs) const; // Based on score_estimate, needed for priority_queue
 */

template <class State>
class AStar {
private:
    unordered_map<string,State> closedStates;
    unordered_map<string,string> keyToPrevKey;
    unordered_map<string,int> keyToPathLength;
    priority_queue<State> openStates; // priority queue allows lookup of largest element by default.
    State bestState;
    double bestStateScore;
    int bestLength;
    bool bestStateFinal;
    bool _finished;
    string initialKey;
public:
    AStar(State initialState) : bestState(initialState) {
        bestStateScore = initialState.scoreEstimate();
        bestLength = 1;
        initialKey = initialState.key();
        bestStateFinal = initialState.final();
        _finished = bestStateFinal;
        closedStates[initialKey] = initialState;
        keyToPathLength[initialKey] = 1;
        if(!_finished) {
            for (State state : initialState.nextStates()) {
                string k = state.key();
                keyToPrevKey[k] = initialKey;
                keyToPathLength[k] = 2;
                openStates.push(state);
            }
        }
    }
    bool finished() {
        return _finished;
    }
    double score() {
        return bestStateScore;
    }
    vector<State> path() {
        vector<State> result;
        State nextState = bestState;
        string nextKey = bestState.key();
        while(true) {
            result.push_back(nextState);
            if(nextKey == initialKey) {
                reverse(result.begin(), result.end());
                return result;
            }
            nextKey = keyToPrevKey[nextKey];
            nextState = closedStates[nextKey];
        }
    }
    void calculateSeconds(double maxSeconds) {
        if(_finished) {
            return;
        }
        clock_t startClock = clock();
        while(true) {
            calculateSteps(50);
            if(_finished) {
                return;
            }
            clock_t currentClock = clock();
            double elapsedSeconds = double(currentClock - startClock) / CLOCKS_PER_SEC;
            if(elapsedSeconds > maxSeconds) {
                return;
            }
        }
    }
    void calculateSteps(int max_steps) {
        for(int i = 0; i < max_steps; i++) {
            if(_finished) {
                return;
            }
            calculateStep();
        }
    }
    void calculateStep() {
        if(_finished) {
            return;
        } else if(openStates.empty()) {
            _finished = true;
            return;
        }
        State state = openStates.top();
        openStates.pop();
        string k = state.key();
        if(closedStates.count(k) > 0) {
            return;
        }
        closedStates[k] = state;
        double stateScore = state.scoreEstimate();
        if(bestStateFinal && stateScore < bestStateScore) {
            _finished = true;
            return;
        }
        int statePathLength = keyToPathLength[k];
        bool stateFinal = state.final();
        bool stateBetterThanBest;
        if(stateFinal && !bestStateFinal) {
            stateBetterThanBest = true;
        } else if(bestStateFinal && !stateFinal) {
            stateBetterThanBest = false;
        } else if(statePathLength > bestLength) {
            stateBetterThanBest = true;
        } else if(bestLength > statePathLength) {
            stateBetterThanBest = false;
        } else {
            stateBetterThanBest = (stateScore > bestStateScore);
        }
        if(stateBetterThanBest) {
            bestState = state;
            bestStateScore = stateScore;
            bestLength = statePathLength;
            bestStateFinal = stateFinal;
        }
        int newLength = statePathLength + 1;
        for (State newState : state.nextStates()) {
            string kNew = state.key();
            if(closedStates.count(kNew) == 0) {
                keyToPrevKey[kNew] = k;
                keyToPathLength[kNew] = newLength;
                openStates.push(newState);
            }
        }
    }
};

#endif
