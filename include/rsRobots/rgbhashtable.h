/* rgbhashtable.h
 *
 * A hashtable for storing RGB values for a Linkbot LED
 * Chaining is used to resolve collisions
 * RGB values need to be stored as an array of 3 values
 * An address of an array of three values is returned for the RGB color values
 *
 * Dawn Hustig-Schultz
 * 2013/12/20
 *
 */

#ifndef RGBHASHTABLE_H_
#define RGBHASHTABLE_H_

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

#endif // RGBHASHTABLE_H_

