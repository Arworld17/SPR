#include <stdio.h>
#include <stdlib.h> //srand，qsort 
#include <string.h>//strcpy 
#include <time.h>//time 

typedef struct {           //别名修饰结构体 
    int snowboard;
    char name[10];
} Person;

void shuffleArray(int arr[], int n) {             //定义函数和随机排列 
    srand(time(0));                               //根据当前时间来设置随机数种子 
int i;
    for ( i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int temp = arr[i];
        arr[i] = arr[j];
        arr[j] = temp;
    }
}

int compare(const void* a, const void* b) {        //定义，进行比较 
    Person* personA = (Person*)a;               //类型转换 
    Person* personB = (Person*)b;
    return personA->snowboard - personB->snowboard;
}

int main() { 
    int snowboards[] = {1, 2, 3, 4};            //赋名 
    Person people[4];
    strcpy(people[0].name, "小刘");
    strcpy(people[1].name, "小周");
    strcpy(people[2].name, "小张");
    strcpy(people[3].name, "小易");

    shuffleArray(snowboards, 4);

    printf("滑雪板分配结果（按照滑雪板从小到大顺序）：\n");
int i;
    for ( i = 0; i < 4; i++) {
        people[i].snowboard = snowboards[i];
    }

    qsort(people, 4, sizeof(Person), compare);               //比较与排序 

    for ( i = 0; i < 4; i++) {
        printf("滑雪板 %d 由 %s 使用\n", people[i].snowboard, people[i].name);
    }

    return 0;
}

