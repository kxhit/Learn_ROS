#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <stack>
#include <vector>
#include <boost/shared_ptr.hpp>


#define MAX_VERTEX_NUM 1000                           //图中最大的节点数目
#define INFINITY 999999
#define NIL -1

typedef struct ArcNode                                          //弧节点，就是邻接链表的表节点
{
     int adjvex;                                                           //该弧所指向尾节点的位置，其实保存的就是数组的下标
     boost::shared_ptr<ArcNode> nextarc;                                            //指向下一条弧的指针
     double weight;                                                          //权重。
}ArcNode;

typedef struct VNode
{
     boost::shared_ptr<ArcNode> firstarc;                                                                   // 弧节点邻接链表的头节点
}VNode,AdjList[MAX_VERTEX_NUM];

typedef struct
{
     AdjList vertices;                                                                                                       //vertices 弧节点列表,每个都是个邻接链表
     int vexnum,arcnum;                                                                                               //vexnum节点数,arcnum弧节点数,即边的数量
}ALGraph;


void minHeapify(int Q[],int d[],int i);
void buildMinHeap(int Q[],int d[]);                                                                                 //建立最小堆
int extractMin(int Q[],int d[]);                                                                                          //从最小队列中，抽取最小结点
void initALGraph(boost::shared_ptr<ALGraph> GPt,int vn);                                        //初始化结点
void insertArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w);               //增加结点边操作
void insertArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w);        //增加结点双边同权重操作
void displayGraph(ALGraph G);                                                                                      //打印结点
int getEdgeWeight(ALGraph G,int vhead,int vtail);                                                       //求边的权重
void initSingleSource(ALGraph G,int s,int d[],int pi[]);                                                 //初始化图每个点
void relax(int u,int v,ALGraph G,int d[],int pi[]);                                                           //松弛操作
void dijkstra(ALGraph G,int s,int d[],int pi[],int Q[]);                                                    //求s为起点的最短路径
void printRoute(int i, int pi[]);                                                                                        //打印i为重点的最短路径
std::vector<int> findMinRoute(ALGraph G,int s,int e,int d[],int pi[],int Q[]);             //寻找最短路径
bool changeArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w);    //改变两个节点双向边的权重
bool changeArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w);           //改变两个节点单边的权重
bool deleteArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail);                      //删除两个节点的单边
bool deleteArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail);                //删除两个节点的双边
#endif // DIJKSTRA_H
