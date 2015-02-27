/* rgbhashtable.c
 *
 * A hashtable for storing RGB values for a Linkbot LED
 * Chaining is used to resolve collisions
 * RGB values need to be stored as an array of 3 values
 * Includes reverse look-up of a string for nearest color match
 *
 *
 * Dawn Hustig-Schultz
 * 2013/12/20
 *
 */

#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <rsRobots/rgbhashtable.h>

//The hash function
unsigned long _hash(rgbHashTable * rgbHT, char *key) {
	unsigned long hash = 5381;
	int i = 0;
	while (hash < ULONG_MAX && i < strlen( key )){
		hash = (hash << 8) + key[i];
		i++;
	}
	return hash % rgbHT->size;
}

//Create the hash table
rgbHashTable* HT_Create() {
	rgbHashTable *rgbHT = (rgbHashTable *)malloc(sizeof(rgbHashTable));
	rgbHT->size = MAX_TABLE_SIZE;
	rgbHT->table = (struct rgbNode **)calloc(1, MAX_TABLE_SIZE * sizeof(struct rgbNode *));

	HT_Add(rgbHT, "aliceBlue", 240, 248, 255);
	HT_Add(rgbHT, "antiqueWhite", 250, 235, 215);
	HT_Add(rgbHT, "aqua", 0, 255, 255);
	HT_Add(rgbHT, "aquamarine", 127, 255, 212);
	HT_Add(rgbHT, "azure", 240, 255, 255);
	HT_Add(rgbHT, "beige", 245, 245, 220);
	HT_Add(rgbHT, "bisque", 255, 228, 196);
	HT_Add(rgbHT, "black", 0, 0, 0);
	HT_Add(rgbHT, "blanchedAlmond", 255, 235, 205);
	HT_Add(rgbHT, "blue", 0, 0, 255);
	HT_Add(rgbHT, "blueViolet", 138, 43, 226);
	HT_Add(rgbHT, "brown", 165, 42, 42);
	HT_Add(rgbHT, "burlyWood", 222, 184, 135);
	HT_Add(rgbHT, "cadetBlue", 95, 158, 160);
	HT_Add(rgbHT, "chartreuse", 127, 255, 0);
	HT_Add(rgbHT, "chocolate", 210, 105, 30);
	HT_Add(rgbHT, "coral", 255, 127, 80);
	HT_Add(rgbHT, "cornflowerBlue", 100, 149, 237);
	HT_Add(rgbHT, "cornSilk", 255, 248, 220);
	HT_Add(rgbHT, "crimson", 220, 20, 60);
	HT_Add(rgbHT, "cyan", 0, 255, 255);
	HT_Add(rgbHT, "darkBlue", 0, 0, 139);
	HT_Add(rgbHT, "darkCyan", 0, 139, 139);
	HT_Add(rgbHT, "darkGoldenrod", 184, 134, 11);
	HT_Add(rgbHT, "darkGray", 169, 169, 169);
	HT_Add(rgbHT, "darkKhaki", 0, 100, 0);
	HT_Add(rgbHT, "darkGreen", 189, 183, 107);
	HT_Add(rgbHT, "darkMagenta", 139, 0, 139);
	HT_Add(rgbHT, "darkOliveGreen", 85, 107, 47);
	HT_Add(rgbHT, "darkOrange", 255, 140, 0);
	HT_Add(rgbHT, "darkOrchid", 153, 50, 204);
	HT_Add(rgbHT, "darkRed", 139, 0, 0);
	HT_Add(rgbHT, "darkSalmon", 233, 150, 122);
	HT_Add(rgbHT, "darkSeaGreen", 143, 188, 143);
	HT_Add(rgbHT, "darkSlateBlue", 72, 61, 139);
	HT_Add(rgbHT, "darkSlateGray", 47, 79, 79);
	HT_Add(rgbHT, "darkTurquoise", 0, 206, 209);
	HT_Add(rgbHT, "darkViolet", 148, 0, 211);
	HT_Add(rgbHT, "deepPink", 255, 20, 147);
	HT_Add(rgbHT, "deepSkyBlue", 0, 191, 255);
	HT_Add(rgbHT, "dimGray", 105, 105, 105);
	HT_Add(rgbHT, "dodgerBlue", 30, 144, 255);
	HT_Add(rgbHT, "fireBrick", 178, 34, 34);
	HT_Add(rgbHT, "floralWhite", 255, 250, 240);
	HT_Add(rgbHT, "forestGreen", 34, 139, 34);
	HT_Add(rgbHT, "fuchsia", 255, 0, 255);
	HT_Add(rgbHT, "gainsboro", 220, 200, 220);
	HT_Add(rgbHT, "ghostWhite", 248, 248, 255);
	HT_Add(rgbHT, "gold", 255, 215, 0);
	HT_Add(rgbHT, "goldenrod", 218, 165, 32);
	HT_Add(rgbHT, "gray", 128, 128, 128);
	HT_Add(rgbHT, "green", 0, 255, 0);
	HT_Add(rgbHT, "greenYellow", 173, 255, 47);
	HT_Add(rgbHT, "honeydew", 240, 255, 240);
	HT_Add(rgbHT, "hotPink", 255, 105, 180);
	HT_Add(rgbHT, "indianRed", 205, 92, 92);
	HT_Add(rgbHT, "indigo", 75, 0, 130);
	HT_Add(rgbHT, "ivory", 255, 255, 240);
	HT_Add(rgbHT, "khaki", 240, 230, 140);
	HT_Add(rgbHT, "lavender", 230, 230, 250);
	HT_Add(rgbHT, "lavenderBlush", 255, 240, 245);
	HT_Add(rgbHT, "lawnGreen", 124, 252, 0);
	HT_Add(rgbHT, "lemonChiffon", 255, 250, 205);
	HT_Add(rgbHT, "lightBlue", 173, 216, 230);
	HT_Add(rgbHT, "lightCoral", 240, 128, 128);
	HT_Add(rgbHT, "lightCyan", 224, 255, 255);
	HT_Add(rgbHT, "lightGoldenrodYellow", 250, 250, 210);
	HT_Add(rgbHT, "lightGray", 211, 211, 211);
	HT_Add(rgbHT, "lightGreen", 144, 238, 144);
	HT_Add(rgbHT, "lightPink", 255, 182, 193);
	HT_Add(rgbHT, "lightSalmon", 255, 160, 122);
	HT_Add(rgbHT, "lightSeaGreen", 32, 178, 170);
	HT_Add(rgbHT, "lightSkyBlue", 135, 206, 250);
	HT_Add(rgbHT, "lightSlateGray", 119, 136, 153);
	HT_Add(rgbHT, "lightSteelBlue", 176, 196, 222);
	HT_Add(rgbHT, "lightYellow", 255, 255, 224);
	HT_Add(rgbHT, "limeGreen", 50, 205, 50);
	HT_Add(rgbHT, "linen", 250, 240, 230);
	HT_Add(rgbHT, "magenta", 255, 0, 255);
	HT_Add(rgbHT, "maroon", 128, 0, 0);
	HT_Add(rgbHT, "mediumAquamarine", 102, 205, 170);
	HT_Add(rgbHT, "mediumBlue", 0, 0, 205);
	HT_Add(rgbHT, "mediumOrchid", 186, 85, 211);
	HT_Add(rgbHT, "mediumPurple", 147, 112, 219);
	HT_Add(rgbHT, "mediumSeaGreen", 60, 179, 113);
	HT_Add(rgbHT, "mediumSlateBlue", 123, 104, 238);
	HT_Add(rgbHT, "mediumSpringGreen", 0, 250, 154);
	HT_Add(rgbHT, "mediumTurquoise", 72, 209, 204);
	HT_Add(rgbHT, "mediumVioletRed", 199, 21, 133);
	HT_Add(rgbHT, "midnightBlue", 25, 25, 112);
	HT_Add(rgbHT, "mintCream", 245, 255, 250);
	HT_Add(rgbHT, "mistyRose", 255, 228, 225);
	HT_Add(rgbHT, "moccasin", 255, 228, 181);
	HT_Add(rgbHT, "navajoWhite", 255, 222, 173);
	HT_Add(rgbHT, "navy", 0, 0, 128);
	HT_Add(rgbHT, "oldLace", 253, 245, 230);
	HT_Add(rgbHT, "olive", 128, 128, 0);
	HT_Add(rgbHT, "oliveDrab", 107, 142, 35);
	HT_Add(rgbHT, "orange", 255, 165, 0);
	HT_Add(rgbHT, "orangeRed", 255, 69, 0);
	HT_Add(rgbHT, "orchid", 218, 112, 214);
	HT_Add(rgbHT, "paleGoldenrod", 238, 232, 170);
	HT_Add(rgbHT, "paleGreen", 152, 251, 152);
	HT_Add(rgbHT, "paleTurquoise", 175, 238, 238);
	HT_Add(rgbHT, "paleVioletRed", 219, 112, 147);
	HT_Add(rgbHT, "papayaWhip", 255, 239, 213);
	HT_Add(rgbHT, "peachPuff", 255, 218, 185);
	HT_Add(rgbHT, "peru", 205, 133, 63);
	HT_Add(rgbHT, "pink", 255, 192, 203);
	HT_Add(rgbHT, "plum", 221, 160, 221);
	HT_Add(rgbHT, "powderBlue", 176, 224, 230);
	HT_Add(rgbHT, "purple", 128, 0, 128);
	HT_Add(rgbHT, "red", 255, 0, 0);
	HT_Add(rgbHT, "rosyBrown", 188, 143, 143);
	HT_Add(rgbHT, "royalBlue", 65, 105, 225);
	HT_Add(rgbHT, "saddleBrown", 139, 69, 19);
	HT_Add(rgbHT, "salmon", 250, 128, 114);
	HT_Add(rgbHT, "sandyBrown", 244, 164, 96);
	HT_Add(rgbHT, "seaGreen", 46, 139, 87);
	HT_Add(rgbHT, "seaShell", 255, 245, 238);
	HT_Add(rgbHT, "sienna", 160, 82, 45);
	HT_Add(rgbHT, "silver", 192, 192, 192);
	HT_Add(rgbHT, "skyBlue", 135, 206, 235);
	HT_Add(rgbHT, "slateBlue", 106, 90, 205);
	HT_Add(rgbHT, "slateGray", 112, 128, 144);
	HT_Add(rgbHT, "snow", 255, 250, 250);
	HT_Add(rgbHT, "springGreen", 0, 255, 127);
	HT_Add(rgbHT, "steelBlue", 70, 130, 180);
	HT_Add(rgbHT, "tan", 210, 180, 140);
	HT_Add(rgbHT, "teal", 0, 128, 128);
	HT_Add(rgbHT, "thistle", 216, 191, 216);
	HT_Add(rgbHT, "tomato", 255, 99, 71);
	HT_Add(rgbHT, "turquoise", 64, 224, 208);
	HT_Add(rgbHT, "violet", 238, 130, 238);
	HT_Add(rgbHT, "wheat", 245, 222, 179);
	HT_Add(rgbHT, "white", 255, 255, 255);
	HT_Add(rgbHT, "whiteSmoke", 245, 245, 245);
	HT_Add(rgbHT, "yellow", 255, 255, 0);
	HT_Add(rgbHT, "yellowGreen", 154, 205, 50);

	return rgbHT;
}

//Destroy the table
void HT_Destroy(rgbHashTable * rgbHT) {
	int i;
	if (!rgbHT) return;

	for (i = 0; i < rgbHT->size; i++) {
		struct rgbNode * n = rgbHT->table[i];
		while (n) {
			struct rgbNode *n_old = n;
			n = n->next;
			free(n_old->key);
			n_old->key = NULL;
			free(n_old);
			n_old = NULL;
		}
	}

	free(rgbHT->table);
	free(rgbHT);
	rgbHT = NULL;
}

//Pass in a key, retrieve a set of RGB values
int HT_Get(rgbHashTable * rgbHT, char * key, int * rgbArray)
{
	unsigned long index;
	struct rgbNode * n = NULL;
	if (!rgbHT) return -1; //If not a valid hash table, then return negative value
	index = _hash(rgbHT, key);
	n = rgbHT->table[index];

	while (n) {
		if (strncmp(key, n->key, MAX_KEYLEN) == 0){
			rgbArray[0] = n->values[0];
			rgbArray[1] = n->values[1];
			rgbArray[2] = n->values[2];
			return 1;	//If the entry exists, return true
		}
		n = n->next;
	}
	return -1;	//If get to this point, entry not in table. Return negative value.
}

//Store a set of RGB values with a specified key
void HT_Add(rgbHashTable *rgbHT, char *key, int r, int g, int b)
{
	unsigned long index;
	struct rgbNode *n_new = NULL;
	int length;

	if (!rgbHT) return;

	index = _hash(rgbHT, key);

	n_new = (struct rgbNode*)calloc(1, sizeof(struct rgbNode));

	n_new->values[0] = r;
	n_new->values[1] = g;
	n_new->values[2] = b;

	length = strlen(key);
	if (length > MAX_KEYLEN){
		length = MAX_KEYLEN;
	}

	n_new->key = (char*)calloc(1, length + 1);

	strcpy(n_new->key, key);

	n_new->next = rgbHT->table[index];
	rgbHT->table[index] = n_new;
}

//Remove a set of RGB values from the table
void HT_Remove(rgbHashTable *rgbHT, char *key)
{
	unsigned long index;
	struct rgbNode *p = NULL;
	struct rgbNode *n = NULL;

	if (!rgbHT) return;

	index = _hash(rgbHT, key);

	n = rgbHT->table[index];

	while (n) {
		if (strncmp(key, n->key, MAX_KEYLEN) == 0) {
			if (p)
				p->next = n->next;

			free (n->key);
			n->key = NULL;

			if (rgbHT->table[index] == n)
				rgbHT->table[index] = NULL;

			free (n);
			n = NULL;
			break;

		}
		p = n;
		n = n->next;
	}
}

//match a set of RGB values with the closest table entry. Then copy this entry's key into "color":
int HT_GetKey(rgbHashTable * rgbHT, int values[], char color[])
{
	int i;
	double distance;
	double shortestDistance;
	char * shortestKey = NULL;

	if (!rgbHT) return -1;
	shortestDistance = 3 * pow((double)255, 2); //Start with max distance, which is the distance between black and white.

	for (i = 0; i < rgbHT->size; i++) {
		struct rgbNode * n = rgbHT->table[i];

		while(n) {
			//Compute square of distance between the RGB values from the LED and the RGB values of the current table entry:
			distance = pow((double)(values[0] - n->values[0]), 2) + pow((double)(values[1] - n->values[1]), 2) + pow((double)(values[2] - n->values[2]), 2);
			//Compare with previously found shortest distance. If current distance is shorter, then update:
			if (distance < shortestDistance){
				shortestDistance = distance;
				//keep track of key:
				shortestKey = n->key;
			}
			n = n->next;
		}
	}
	if (strcmp("aqua", shortestKey) == 0){
		strcpy(color, "cyan");
	}
	else if (strcmp("fuchsia", shortestKey) == 0){
		strcpy(color, "magenta");
	}
	else{
		strcpy(color, shortestKey);
	}
	return 0;
}
