#ifndef RSROBOTS_RGBHASHTABLE_H_
#define RSROBOTS_RGBHASHTABLE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_TABLE_SIZE 150
#define MAX_KEYLEN 25

struct rgbNode{
	char *key;
	int values[3];
	struct rgbNode *next;
};

typedef struct rgbHashTable {
	struct rgbNode **table;
	int size;
} rgbHashTable;

rgbHashTable* HT_Create(void);						// create the table
void HT_Destroy(rgbHashTable*);						// destroy the table
int HT_Get(rgbHashTable*, char*, int*);				// retrieve entry
void HT_Add(rgbHashTable*, char*, int, int, int);	// store entry
void HT_Remove(rgbHashTable*, char*);				// remove entry
int HT_GetKey(rgbHashTable*, int[], char[]);		// reverse look-up

#ifdef __cplusplus
}
#endif

#endif // RSROBOTS_RGBHASHTABLE_H_

