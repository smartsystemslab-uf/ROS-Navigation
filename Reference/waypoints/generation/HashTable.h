//-----------------------------------------------------------
//  Purpose:    Header file for the HashTable class.
//  Author:     John Gauch
//-----------------------------------------------------------

#include <iostream>
#include <vector>
using namespace std;
const int NONE = 0;
const int EMPTY = -1;
const int DELETED = -2;

//-----------------------------------------------------------
// Define the HashTable class interface
//-----------------------------------------------------------
class HashTable {
  public:
    // Constructors
    HashTable(int size);
    HashTable(const HashTable & ht);
    ~HashTable();

    // Methods
    bool Insert(int key, int value);
    bool Search(int key, int &value);
    bool SearchMaxNeighbor(int key, int &value, int neighborhood, int &r, int &c);
    bool Delete(int key);
    vector<int> getPointsOfX(int X);
    void Print();

  private:
    // Private methods
    int Hash(int key);
    int Hash2(int index);

    // Private data
    int Size;
    int *Value;
    int *Key;
    int insertions;
};
