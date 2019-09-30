#include <iostream>
#include <vector>

using namespace std;


class TransitionPoint {
public:
	T m_stateInfo;
	double m_cost;
	double m_costSoFar;
	State *m_parent;

	State() {
		m_cost = 0;
		m_costSoFar = 0;
		m_parent = NULL;
	}
	State(T s_info) :
			m_stateInfo(s_info) {
		m_cost = 0;
		m_costSoFar = 0;
		m_parent = NULL;
	}
	~State() {
	}

	inline void findNeighbors(std::vector<State>& neighbors) {
		std::vector<T> neighbors_tmp;
		m_stateInfo.findNeighbors(neighbors_tmp);
		neighbors.resize(0);
		for (unsigned int i = 0; i < neighbors_tmp.size(); i++) {
			State s(neighbors_tmp[i]);
			neighbors.push_back(s);
		}
	}
	inline void rebuildPath() {
		State<T> *aux = this;
		while (aux != NULL) {
			aux->m_stateInfo.rebuildPath();
			aux = aux->m_parent;
		}
	}

	inline bool operator<(const State& other) const {
		if (m_cost < other.m_cost)
			return true;
		else
			return false;
	}

	inline bool operator==(const State& other) const {
		return (m_stateInfo == other.m_stateInfo);
	}

	friend ostream &operator<<(ostream &output, const State &S) {
		output << "Cost : " << S.m_cost << " " << S.m_stateInfo;
		return output;
	}

};

