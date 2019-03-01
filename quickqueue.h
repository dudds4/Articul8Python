#ifndef QUICKQUEUE_H
#define QUICKQUEUE_H

#include <vector>

template<typename T>
struct QuickQueue
{
	
private:
	typedef std::vector<T> List;

public:
	unsigned size() { return list.size() - i; }
	unsigned fullSize() { return list.size(); } 

	T getNext() { return list.at(i++); checkMoveHead(); }
	
	void push_back(T p) { list.push_back(p); checkMoveHead(); }

	void push_back(List l) { 
		for(auto p : l)
			list.push_back(p);

		checkMoveHead();
	}

private:

	void checkMoveHead() { if(fullSize() > 2 * size()) moveHead(); }

	void moveHead()
	{
		list.erase(list.begin(), list.begin() + i);
		i = 0;
	}

	List list;
	unsigned i = 0;
};

#endif