//-----------------------------------------------------------
//  Purpose:    Implementation of HashTable class.
//  Author:     John Gauch
//-----------------------------------------------------------
#include "HashTable.h"

//-----------------------------------------------------------
// Constructor
//-----------------------------------------------------------
HashTable::HashTable(int size) {
  Size = size;
  Value = new int[Size];
  Key = new int[Size];
  insertions = 0;

  for (int index=0; index < Size; index++) {
    Value[index] = NONE;
    Key[index] = EMPTY;
  }
}

//-----------------------------------------------------------
// Copy constructor
//-----------------------------------------------------------
HashTable::HashTable(const HashTable & ht) {
  Size = ht.Size;
  Value = new int[Size];
  Key = new int[Size];
  insertions = ht.insertions;

  for (int index=0; index < Size; index++) {
    Value[index] = ht.Value[index];
    Key[index] = ht.Key[index];
  }
}

//-----------------------------------------------------------
// Destructor
//-----------------------------------------------------------
HashTable::~HashTable() {
  delete []Value;
  delete []Key;
}

//-----------------------------------------------------------
// Insert method
//-----------------------------------------------------------
bool HashTable::Insert(int key, int value) {
  if (insertions >= Size) {
    cout << "Hash Table is full. You need to increase the size." << endl;
    return false;
  }
  // Find desired key
  int index = Hash(key);
  while ((Key[index] != key) && (Key[index] != EMPTY))
    index = Hash2(index);

  // Insert value into hash table
  Value[index] = value;
  Key[index] = key;
  insertions++;
  return true;
}

//-----------------------------------------------------------
// Search method
//-----------------------------------------------------------
bool HashTable::Search(int key, int &value) {
  // Find desired key
  int index = Hash(key);
  while ((Key[index] != key) && (Key[index] != EMPTY))
    index = Hash2(index);

  // Return value from hash table
  if (Key[index] == key)
    value = Value[index];
  return (Key[index] == key);
}

//-----------------------------------------------------------
// Search for Max Neighbor method // note: it is about 6 pixels per inch @ haas hall. allow for 4 cm size between waypoints
//-----------------------------------------------------------
bool HashTable::SearchMaxNeighbor(int key, int &value, int neighborhood, int &r, int &c) {
  // Find desired key
  bool ret = false;
  int desiredVal = value;
  int bestVal = 0;
  int rowStart  = max(key / 1000 - neighborhood, 0);
  int rowFinish = min(key / 1000 + neighborhood, 479);
  int colStart  = max(key % 1000 - neighborhood, 0);
  int colFinish = min(key % 1000 + neighborhood, 639);

  for (size_t curRow = rowStart; curRow <= rowFinish; curRow++) {
    for (size_t curCol = colStart; curCol <= colFinish; curCol++) {
      int tmpKey = curCol + curRow * 1000;
      int index = Hash(tmpKey);
      while ((Key[index] != tmpKey) && (Key[index] != EMPTY))
        index = Hash2(index);
      if (Key[index] == tmpKey && Value[index] > bestVal) {
        ret = true; // I have found a neighbor that has been inserted before, so true
        value = Value[index];
        r = curRow;
        c = curCol;
      }
    }
  }
  if (desiredVal > value) {
    int index = Hash(key);
    while ((Key[index] != key) && (Key[index] != EMPTY))
      index = Hash2(index);
    // Return value from hash table
    if (Key[index] == key) {
      ret = true; // My key is already in the table, so true
      value = Value[index];
    }
    r = key / 1000;
    c = key % 1000;
  }
  return ret; // This returns true if the passed key is in the table or any of its valid neighbors
  // return (Key[index] == key);
}

//-----------------------------------------------------------
// Delete method
//-----------------------------------------------------------
bool HashTable::Delete(int key) {
  // Find desired key
  int index = Hash(key);
  while ((Key[index] != key) && (Key[index] != EMPTY))
    index = Hash2(index);

  // Delete value from hash table
  if (Key[index] == key) {
    Value[index] = NONE;
    Key[index] = DELETED;
    return true;
  }
  return false;
}

//-----------------------------------------------------------
// Primary hash function
//-----------------------------------------------------------
int HashTable::Hash(int key) {
  // return (key / 1000 + key % 1000) % Size;
  return key % Size;
}

//-----------------------------------------------------------
// Secondary hash function (linear probing)
//-----------------------------------------------------------
int HashTable::Hash2(int index) {
  return (index+1) % Size;
}

//-----------------------------------------------------------
// Returns a vector of the keys whose value is greater than X
//-----------------------------------------------------------
vector<int> HashTable::getPointsOfX(int X) {
  vector<int> ret;
  for (int index=0; index < Size; index++) {
    if (Value[index] > X) {
      ret.push_back(Key[index]);
    }
  }
  return ret;
}

//-----------------------------------------------------------
// Print function for debugging
//-----------------------------------------------------------
void HashTable::Print() {
  cout << "Row\t" << "Col\t" << "Value\n";
  for (int index=0; index < Size; index++) {
    if (Key[index] > 0) {
      cout << Key[index] / 1000 << "\t" << Key[index] % 1000 << "\t" << Value[index] << "\n";
    }
  }
}

//-----------------------------------------------------------
// Main program.
//-----------------------------------------------------------
// int main() {
// 	srandom(time(NULL));
//    // ADD HERE
//   int size, count, max;
// 	cout << "enter size" << endl;
// 	cin >> size;
// 	cout << "enter count" << endl;
// 	cin >> count;
// 	cout << "enter max" << endl;
// 	cin >> max;
// 	HashTable hashT(size);
// 	for (int i = 0; i < count; i++) {
// 		hashT.Insert(random() % max, random() % max);
// 	}
// 	hashT.Print();
// }
