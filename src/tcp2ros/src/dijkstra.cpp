#include <iostream>
#include "tcp2ros/dijkstra.h"

using namespace std;

//保持最小堆的性质
void minHeapify(int Q[],int d[],int i)
{
    //smallest,l,r,i都是优先队列元素的下标，范围是从1 ~ heap-size[Q]
     int l = 2*i;
     int r = 2*i+1;
     int smallest;
     if(l<=Q[0] && d[ Q[l] ] < d[ Q[i] ])
     {
        smallest = l;
     }
     else
     {
        smallest = i;
     }
     if(r<=Q[0] && d[ Q[r] ] < d[ Q[smallest] ])
     {
        smallest = r;
     }
     if(smallest!=i)
     {
        int temp = Q[i];
        Q[i] = Q[smallest];
        Q[smallest] = temp;

        minHeapify(Q,d,smallest);
     }
}
//建堆(O（N）)
void buildMinHeap(int Q[],int d[]) //建立最小堆
{
     for(int i=Q[0]/2;i>=1;i--)
     {
        minHeapify(Q,d,i); //调用minHeapify，以保持堆的性质。
     }
}
//从最小队列中，抽取最小结点的工作了
int extractMin(int Q[],int d[])   //3、从最小队列中，抽取最小结点
{
     //摘取优先队列中最小元素的内容，这里返回图中顶点的标号(0 ~ G.vexnum-1)，
     //这些标号是保存在Q[1..n]中的
     if(Q[0]<1)
     {
      cout<<"heap underflow!"<<endl;
      return -10000;
     }
     int min = Q[1];
     Q[1] = Q[Q[0]];
     Q[0] = Q[0] - 1;
     minHeapify(Q,d,1);
     return min;
}
//初始化结点
void initALGraph(boost::shared_ptr<ALGraph> GPt,int vn)
{
     GPt->arcnum = 0;
     GPt->vexnum = vn;
     /*
     for(int i=0;i<vn;i++)
     {
          GPt->vertices[i].firstarc = NULL;
     }
    */
}
//增加结点边操作
void insertArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w)
{
     //ArcNode* arcNodePt = new ArcNode;
     boost::shared_ptr<ArcNode> arcNodePt(new ArcNode());
     //arcNodePt->nextarc = NULL;
     arcNodePt->adjvex = vtail;
     arcNodePt->weight = w;

     boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
     if(tailPt==NULL)
     {
          GPt->vertices[vhead].firstarc = arcNodePt;
     }
     else
     {
          while(tailPt->nextarc!=NULL)
          {
               tailPt = tailPt->nextarc;
          }
          tailPt->nextarc = arcNodePt;
     }
     GPt->arcnum ++;
}
//增加结点双边同权重操作
void insertArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w)
{
     boost::shared_ptr<ArcNode> arcNodePt1(new ArcNode());
     //arcNodePt->nextarc = NULL;
     arcNodePt1->adjvex = vtail;
     arcNodePt1->weight = w;

     boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
     if(tailPt==NULL)
     {
          GPt->vertices[vhead].firstarc = arcNodePt1;
     }
     else
     {
          while(tailPt->nextarc!=NULL)
          {
               tailPt = tailPt->nextarc;
          }
          tailPt->nextarc = arcNodePt1;
     }
     GPt->arcnum ++;

     boost::shared_ptr<ArcNode> arcNodePt2(new ArcNode());
     //arcNodePt->nextarc = NULL;
     arcNodePt2->adjvex = vhead;
     arcNodePt2->weight = w;

     boost::shared_ptr<ArcNode> headPt = GPt->vertices[vtail].firstarc;
     if(headPt==NULL)
     {
          GPt->vertices[vtail].firstarc = arcNodePt2;
     }
     else
     {
          while(headPt->nextarc!=NULL)
          {
               headPt = headPt->nextarc;
          }
          headPt->nextarc = arcNodePt2;
     }
     GPt->arcnum ++;
}
bool changeArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w)
{
    boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
    while(tailPt!=NULL)
    {
        if(tailPt->adjvex==vtail)
            break;
        else
            tailPt = tailPt->nextarc;
    }
    if(tailPt!=NULL)
    {
        tailPt->weight=w;
        return 1;
    }
    else
        return 0;
}
bool changeArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail,int w)
{
    int succed1=0,succed2=0;
    boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
    while(tailPt!=NULL)
    {
        if(tailPt->adjvex==vtail)
            break;
        else
            tailPt = tailPt->nextarc;
    }
    if(tailPt!=NULL)
    {
        tailPt->weight=w;
        succed1=1;
    }
    else
        return 0;

    boost::shared_ptr<ArcNode> headPt = GPt->vertices[vtail].firstarc;
    while(headPt!=NULL)
    {
        if(headPt->adjvex==vhead)
            break;
        else
            headPt = headPt->nextarc;
    }
    if(headPt!=NULL)
    {
        headPt->weight=w;
        succed2=1;
    }
    else
        return 0;

    if(succed1&&succed2)
        return 1;
    else
        return 0;
}
bool deleteArc(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail)
{
    boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
    boost::shared_ptr<ArcNode> tempPt = GPt->vertices[vhead].firstarc;
    while(tailPt!=NULL)
    {
        if(tailPt->adjvex==vtail)
            break;
        else
        {
            tempPt=tailPt;
            tailPt = tailPt->nextarc;
        }
    }
    if(tailPt!=NULL&&tailPt->adjvex==vtail)
    {
        if(tempPt==tailPt)
        {
            GPt->vertices[vhead].firstarc=tailPt->nextarc;
            tailPt.reset();
            tempPt.reset();
        }
        else
        {
            tempPt->nextarc=tailPt->nextarc;
            tailPt.reset();
        }
        GPt->arcnum --;
        return 1;
    }
    else
        return 0;
}

bool deleteArcTwo(boost::shared_ptr<ALGraph> GPt,int vhead,int vtail)
{
    int succed1=0,succed2=0;
    boost::shared_ptr<ArcNode> tailPt = GPt->vertices[vhead].firstarc;
    boost::shared_ptr<ArcNode> headPt = GPt->vertices[vtail].firstarc;
    boost::shared_ptr<ArcNode> tempPt = GPt->vertices[vhead].firstarc;
    while(tailPt!=NULL)
    {
        if(tailPt->adjvex==vtail)
            break;
        else
        {
            tempPt=tailPt;
            tailPt = tailPt->nextarc;
        }
    }
    if(tailPt!=NULL&&tailPt->adjvex==vtail)
    {
        if(tempPt==tailPt)
        {
            GPt->vertices[vhead].firstarc=tailPt->nextarc;
            tailPt.reset();
            tempPt.reset();
        }
        else if(!tailPt->nextarc)
        {
            tailPt.reset();
            boost::shared_ptr<ArcNode> nummPt;
            tempPt->nextarc=nummPt;
        }
        else
        {
            tempPt->nextarc=tailPt->nextarc;
            tailPt.reset();
        }
        GPt->arcnum --;
        succed1=1;
    }
    else
        return 0;

    tempPt = GPt->vertices[vtail].firstarc;
    while(headPt!=NULL)
    {
        if(headPt->adjvex==vhead)
            break;
        else
        {
            cout<<"i"<<endl;
            tempPt=headPt;
            headPt = headPt->nextarc;
        }
    }

    if(headPt!=NULL&&headPt->adjvex==vhead)
    {
        if(tempPt==headPt)
        {
            GPt->vertices[vtail].firstarc=headPt->nextarc;
            headPt.reset();
            tempPt.reset();
        }
        else if(!headPt->nextarc)
        {
            headPt.reset();
            boost::shared_ptr<ArcNode> nummPt;
            tempPt->nextarc=nummPt;
        }
        else
        {
            tempPt->nextarc=headPt->nextarc;
            headPt.reset();
        }
        GPt->arcnum --;
        succed2=1;
    }
    else
        return 0;

    if(succed1&&succed2)
        return 1;
}
//打印结点
void displayGraph(ALGraph G)
{
     boost::shared_ptr<ArcNode> arcNodePt;
     for(int i=0;i<G.vexnum;i++)
     {
          arcNodePt = G.vertices[i].firstarc;
          cout<<"vertex"<<i<<": ";
          while(arcNodePt!=NULL)
          {
               cout<<arcNodePt->adjvex<<"("<<"weight:"<<arcNodePt->weight<<")"<<" ";
               arcNodePt = arcNodePt->nextarc;
          }
          cout<<endl;
     }
}
//求边的权重
int getEdgeWeight(ALGraph G,int vhead,int vtail)
{
     boost::shared_ptr<ArcNode> arcNodePt = G.vertices[vhead].firstarc;
     while(arcNodePt!=NULL)
     {
          if(arcNodePt->adjvex==vtail)
          {
               return arcNodePt->weight;
          }
          arcNodePt = arcNodePt->nextarc;
     }
     return INFINITY;
}

void initSingleSource(ALGraph G,int s,int d[],int pi[])
{
    //1、初始化结点工作
    for(int i=0;i<G.vexnum;i++)
    {
        d[i] = INFINITY;
        pi[i] = NIL;
    }
    d[s] = 0;
}

void relax(int u,int v,ALGraph G,int d[],int pi[])
{
     //4、松弛操作。
     //u是新加入集合S的顶点的标号
     if(d[v]>d[u]+getEdgeWeight(G,u,v))
     {
        d[v] = d[u] + getEdgeWeight(G,u,v);
        pi[v] = u;
     }
}
/*  G:graph
 *  s: node's num,
 *  d: path distance
 *  Q: Minmist list
 *  pi: path stack
 */
void dijkstra(ALGraph G,int s,int d[],int pi[],int Q[])
{
     //Q[]是最小优先队列，Q[1..n]中存放的是图顶点标号,Q[0]中存放堆的大小
     //优先队列中有key的概念，这里key可以从d[]中取得。比如说，Q[2]的大小(key)为 d[ Q[2] ]

    initSingleSource(G,s,d,pi);//1、初始化结点工作

    //2 初始化队列,
    Q[0]=G.vexnum;
    for (int i=1;i<=Q[0];i++)
    {
        Q[i]=i-1;
    }

    Q[1]=s;
    Q[s+1]=0;

    int u;
    int v;
    while(Q[0]!=0)
    {
        buildMinHeap(Q,d);          //建立最小堆
        u=extractMin(Q,d);           //从最小队列中，抽取最小结点

        //对在 s 和 t 之间寻找一条最短路径的话，我们可以在第9行添加条件如果满足 u = t 的话终止程序


        boost::shared_ptr<ArcNode> arcNodePt=G.vertices[u].firstarc;
        while(arcNodePt!=NULL)
        {
            v = arcNodePt->adjvex;
            relax(u,v,G,d,pi);    //4、松弛操作。
            arcNodePt = arcNodePt->nextarc;
        }
    }
}
//打印路径,反着打印
void printRoute(int i, int pi[])
{
    int nod=i;
    cout<<i;
    while(pi[nod]!=-1)
    {
        cout<<"<-"<<pi[nod];
        nod=pi[nod];
    }
    cout<<endl;
}

std::vector<int> findMinRoute(ALGraph G,int s,int e,int d[],int pi[],int Q[])
{
    //Q[]是最小优先队列，Q[1..n]中存放的是图顶点标号,Q[0]中存放堆的大小
    //优先队列中有key的概念，这里key可以从d[]中取得。比如说，Q[2]的大小(key)为 d[ Q[2] ]

   initSingleSource(G,s,d,pi);//1、初始化结点工作

   //2 初始化队列,
   Q[0]=G.vexnum;
   for (int i=1;i<=Q[0];i++)
   {
       Q[i]=i-1;
   }

   Q[1]=s;
   Q[s+1]=0;

   int u;
   int v;
   while(Q[0]!=0)
   {
       buildMinHeap(Q,d);          //建立最小堆
       u=extractMin(Q,d);           //从最小队列中，抽取最小结点

       //对在 s 和 t 之间寻找一条最短路径的话，我们可以在第9行添加条件如果满足 u = t 的话终止程序
       if(u==e)
            break;

       boost::shared_ptr<ArcNode> arcNodePt=G.vertices[u].firstarc;
       while(arcNodePt!=NULL)
       {
           v = arcNodePt->adjvex;
           relax(u,v,G,d,pi);    //4、松弛操作。
           arcNodePt = arcNodePt->nextarc;
       }
   }

    //输出路径到stack
   int nod=e;
   vector<int> route;
   route.push_back(e);
   while(pi[nod]!=-1)
   {
       route.push_back(pi[nod]);
       nod=pi[nod];
   }
   return route;
}
