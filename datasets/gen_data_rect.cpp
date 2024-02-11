#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
#include <time.h>
#include <stdlib.h>
#include <map>
#include <conio.h>
#include <unordered_set>
#include <unordered_map>
#include <unistd.h>

FILE* fin;
FILE* fout;


using namespace std;

#define pi (2*acos(0.0))
#define eps 0.0000001

#define N 120
#define cpx 4
#define numobs 60
#define rrange (N * cpx)
#define crange 40
#define osa 35
#define osd 25
#define numtarget 10
#define A 4

int G[N][N];
bool B[N][N];
int T;

vector< vector<int> > AL;


double max2(double a,double b)
{
    return (a>b)?a:b;
}

double min2(double a,double b)
{
    return (a<b)?a:b;
}


class point
{
public:
    double x,y;
    point(double xx=0.0, double yy=0.0)
    {
        x=xx;
        y=yy;
    }
};

class cell
{
public:
    int x, y;
    point c;
    cell(int xx = 0.0, int yy = 0.0)
    {
        x = xx;
        y = yy;
        c.x = (x + 0.5) * cpx;
        c.y = (y + 0.5) * cpx;
    }
};

class obs
{
public:
    double xl,xh,yl,yh;
    obs(double xxl=0.0, double xxh=0.0, double yyl=0.0, double yyh=0.0)
    {
        xl=xxl;
        xh=xxh;
        yl=yyl;
        yh=yyh;
    }
};


class robot
{
public:
    int ind;
    int cnt;
    robot(int i = -1, int c = 0)
    {
        ind = i;
        cnt = c;
    }
};

bool comparator(const robot& lhs, const robot& rhs)
{
   return lhs.cnt < rhs.cnt;
}

bool rcomparator(const robot& lhs, const robot& rhs)
{
   return lhs.cnt > rhs.cnt;
}


vector<obs> obsdata;
vector<cell> targetdata;
vector< vector<cell> > trajdata;
vector<int> targetdir = vector<int>(numtarget, 0);
vector<int> dirs = vector<int>(numtarget, 0);


void po(obs o)
{
    cout<<o.xl<<" "<<o.xh<<" "<<o.yl<<" "<<o.yh<<endl;
}

bool doesint1(obs a,obs b)
{
    if(((b.xl+eps<a.xl&&a.xl+eps<b.xh)||(b.xl+eps<a.xh&&a.xh+eps<b.xh)||(a.xl+eps<b.xl&&b.xl+eps<a.xh)||(a.xl+eps<b.xh&&b.xh+eps<a.xh))&&((b.yl+eps<a.yl&&a.yl+eps<b.yh)||(b.yl+eps<a.yh&&a.yh+eps<b.yh)||(a.yl+eps<b.yl&&b.yl+eps<a.yh)||(a.yl+eps<b.yh&&b.yh+eps<a.yh)))return true;

    return false;
}

bool doesintn(vector<obs> ol,obs o)
{
    int i;
    for(i=0;i<ol.size();i++)if(doesint1(ol[i],o)==true)return true;

    return false;
}

bool isinside1(obs o, point p)
{
    if(o.xl<p.x+eps&&p.x<o.xh+eps&&o.yl<p.y+eps&&p.y<o.yh+eps)return true;

    return false;
}

bool isinsiden(vector<obs> ol, point p)
{
    int i;
    for(i=0;i<ol.size();i++)if(isinside1(ol[i],p)==true)return true;

    return false;
}




void genobs()
{
    obsdata.clear();

    obs o;
    int i,ri,rii;
    double rd;
    srand(time(NULL));
    int cnt=numobs;
    while(cnt!=0)
    {
        ri=rand()%(rrange-osa-osd);
        rii=rand()%1000;
        o.xl=ri+(double)rii/1000.0;
        ri=rand()%(2*osd)+(osa-osd);
        rii=rand()%1000;
        o.xh=o.xl+ri+(double)rii/1000.0;

        ri=rand()%(rrange-osa-osd);
        rii=rand()%1000;
        o.yl=ri+(double)rii/1000.0;
        ri=rand()%(2*osd)+(osa-osd);
        rii=rand()%1000;
        o.yh=o.yl+ri+(double)rii/1000.0;


        if(doesintn(obsdata,o)==false)
        {
            obsdata.push_back(o);
            cnt--;
        }
    }
}

void genblockedcells()
{
    int i, j, k;

	for (i = 0; i < N; i++)
    {
        for (j = 0; j < N; j++)
        {
            point tp;
            tp.x = i * cpx + (cpx / 2.0);
            tp.y = j * cpx + (cpx / 2.0);

            bool flag = true;
            for (k = 0; k < numobs; k++)
            {
                if (isinside1(obsdata[k], tp) == true)
                {
                    flag = false;
                    break;
                }
            }
            B[i][j] = flag;
        }
    }
}

void gentarget()
{
    targetdata.clear();

    int cnt = numtarget;
    while(cnt != 0)
    {
        int cx = rand() % N;
        int cy = rand() % N;

        if (B[cx][cy] == true)
        {
            targetdata.push_back(cell(cx, cy));
            cnt--;
        }

    }
}

void gentraj()
{
    trajdata.clear();
    for (int i = 0; i < numtarget; i++) trajdata.push_back(vector<cell>(4, cell(0, 0)));
}


double area3p(point a, point b, point c)
{
    return a.x*(b.y-c.y)+b.x*(c.y-a.y)+c.x*(a.y-b.y);
}

bool doesint(point a, point b, point c, point d)
{
    if(area3p(a,b,c)*area3p(a,b,d)<=0&&area3p(c,d,a)*area3p(c,d,b)<=0)
        return true;
    return false;
}

vector<obs> rangequery(point p)
{
    int j;
    vector<obs> ret;
    obs to;
    to.xl=p.x-crange;
    to.xh=p.x+crange;
    to.yl=p.y-crange;
    to.yh=p.y+crange;
    for(j=0;j<obsdata.size();j++)
    {
        if(doesint1(obsdata[j],to)==true)ret.push_back(obsdata[j]);
    }
    return ret;
}


vector<int> shift(vector<int> V)
{
    for (int i = 1; i < V.size(); i++) V[i] += i * 4;
    return V;
}


//select from robots with S[]=-1 for which incremental coverage is maximized
void greedy_next(vector<int>& S, unordered_set<int>& TC)
{
    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < numtarget; i++)
    {
        if (S[i] == -1)
        {
            for (int j = 0; j < 4; j++)
            {
                int cnt = 0;
                cell tmp = trajdata[i][j];
                int cl = tmp.x * N + tmp.y;
                for (int k = 0; k < AL[cl].size(); k++)
                {
                    auto it = TC.find(AL[cl][k]);
                    if (it == TC.end()) cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
                }
                if (cnt > max_coverage)
                {
                    max_coverage = cnt;
                    max_index = i * 4 + j;
                }
            }
        }
    }
    S[max_index / 4] = max_index % 4;
    cell tmp = trajdata[max_index / 4][max_index % 4];
    int cl = tmp.x * N + tmp.y;
    for (int i = 0; i < AL[cl].size(); i++) TC.insert(AL[cl][i]);
}


//select the robot not in F which has the maximum number of targets which none of the other robots covers
void greedy_next_general_better(vector<int> S, vector<int>& F)
{
    unordered_map<int, int> C;
    C.clear();

    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            cell tmp = trajdata[S[i] / 4][S[i] % 4];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                auto it = C.find(AL[cl][k]);
                if (it == C.end()) C[AL[cl][k]] = 1;
                else C[AL[cl][k]] += 1;
            }
        }
    }

    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            int cnt = 0;
            cell tmp = trajdata[S[i] / 4][S[i] % 4];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                if (C[AL[cl][k]] == 1) cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = S[i];
            }
        }
    }
    F[max_index / 4] = 1;
}


//select next robot not in F such that incremental coverage is maximized
void greedy_next_general(vector<int> S, vector<int>& F, unordered_set<int>& TC)
{
    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            int cnt = 0;
            cell tmp = trajdata[S[i] / 4][S[i] % 4];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                auto it = TC.find(AL[cl][k]);
                if (it == TC.end()) cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = S[i];
            }
        }
    }
    F[max_index / 4] = 1;

    cell tmp = trajdata[max_index / 4][max_index % 4];
    int cl = tmp.x * N + tmp.y;
    for (int i = 0; i < AL[cl].size(); i++) TC.insert(AL[cl][i]);
}


//calculate the coverage of the robots not in D
int differential_coverage(vector<int> S, vector<int> D)
{
    unordered_set<int> V;
    V.clear();
    for (int i = 0; i < S.size(); i++)
    {
        if (D[i] == 0)
        {
            cell tmp = trajdata[S[i] / 4][S[i] % 4];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++) V.insert(AL[cl][k]);
        }
    }


    int coverage = 0;
    for (auto it = V.begin(); it != V.end(); it++)
    {
        coverage += 100 - G[*it / N][*it % N];
    }

    return coverage;
}


//old paper implementation
int old_paper()
{
    vector<int> S1 = vector<int>(numtarget, -1);
    for (int i = 0; i < numtarget; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < 4; j++)
        {
            int cnt = 0;
            cell tmp = trajdata[i][j];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = j;
            }
        }
        S1[i] = max_index;
    }

    vector<int> V = vector<int>(numtarget, 0);
    for (int i = 0; i < A; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < numtarget; j++)
        {
            if (V[j] == 0)
            {
                int cnt = 0;
                cell tmp = trajdata[j][S1[j]];
                int cl = tmp.x * N + tmp.y;
                for (int k = 0; k < AL[cl].size(); k++)
                {
                    cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
                }

                if (cnt > max_coverage)
                {
                    max_coverage = cnt;
                    max_index = j;
                }
            }
        }
        V[max_index] = 1;
    }


    vector<int> S2 = vector<int>(numtarget, -1);

    for (int i = 0; i < numtarget; i++)
    {
        if (V[i] == 1)
        {
            S2[i] = S1[i];
        }
    }

    unordered_set<int> TC;
    TC.clear();
    for (int i = A; i < numtarget; i++)
    {
        greedy_next(S2, TC);
    }


    vector<int> F = vector<int>(numtarget, 0);
    TC.clear();
    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(S2), F);
    }
    return differential_coverage(shift(S2), F);
}



//oblivious Greedy
//for each robot, select the path with highest targets
int oblivious_greedy()
{
    vector<int> S1 = vector<int>(numtarget, -1);
    for (int i = 0; i < numtarget; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < 4; j++)
        {
            int cnt = 0;
            cell tmp = trajdata[i][j];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = j;
            }
        }
        S1[i] = max_index;
    }

    vector<int> F = vector<int>(numtarget, 0);
    unordered_set<int> TC;
    TC.clear();

    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(S1), F);
    }
    return differential_coverage(shift(S1), F);
}


//given an ordering of the robots, make R greedy choices to maximize incremental target coverage
vector<int> robot_permutation(vector<int> S)
{
    unordered_set<int> TC;
    TC.clear();

    vector<int> SQ = vector<int>(numtarget, 0);


    for (int i = 0; i < S.size(); i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < 4; j++)
        {
            int cnt = 0;
            cell tmp = trajdata[S[i]][j];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                auto it = TC.find(AL[cl][k]);
                if (it == TC.end()) cnt += 100 - G[AL[cl][k] / N][AL[cl][k] % N];
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = j;
            }

        }
        SQ[S[i]] = max_index;
        cell tmp = trajdata[S[i]][max_index];
        int cl = tmp.x * N + tmp.y;

        for (int j = 0; j < AL[cl].size(); j++) TC.insert(AL[cl][j]);
    }
    return SQ;
}


//ordered greedy - robots are ordered by increasing cardinality of union of target coverage
int ordered_greedy_cu()
{
    vector<robot> RC;
    for (int i = 0; i < numtarget; i++)
    {
        unordered_set<int> U;
        U.clear();

        for (int j = 0; j < 4; j++)
        {
            cell tmp = trajdata[i][j];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                U.insert(AL[cl][k]);
            }
        }

        int cnt = 0;
        for (auto it = U.begin(); it != U.end(); it++) cnt += 100 - G[*it / N][*it % N];

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &comparator);



    vector<int> PR = vector<int>(numtarget, 0);
    vector<int> RES;

    for (int i = 0; i < numtarget; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);
    dirs = RES;


    vector<int> F = vector<int>(numtarget, 0);
    unordered_set<int> TC;
    TC.clear();

    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(RES), F);
    }
    return differential_coverage(shift(RES), F);

}



//initialized using ordered greedy, attacker uses traditional greedy
int local_search_iog_atg()
{
    vector<robot> RC;
    for (int i = 0; i < numtarget; i++)
    {
        unordered_set<int> U;
        U.clear();

        for (int j = 0; j < 4; j++)
        {
            cell tmp = trajdata[i][j];
            int cl = tmp.x * N + tmp.y;
            for (int k = 0; k < AL[cl].size(); k++)
            {
                U.insert(AL[cl][k]);
            }
        }

        int cnt = 0;
        for (auto it = U.begin(); it != U.end(); it++) cnt += 100 - G[*it / N][*it % N];

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &comparator);



    vector<int> PR = vector<int>(numtarget, 0);
    vector<int> RES;

    for (int i = 0; i < numtarget; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);


    vector<int> S1 = RES;

    vector<int> F = vector<int>(numtarget, 0);
    unordered_set<int> TC;
    TC.clear();

    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(S1), F);
    }


    vector<int> SOL = S1;
    int sol_value = differential_coverage(shift(S1), F);












    while (true)
    {
        bool flag = false;
        for (int i = 0; i < numtarget * 4; i++)
        {
            int rn = i / 4;
            int pn = i % 4;
            if (SOL[rn] != pn)
            {
                vector<int> TS = SOL;
                TS[rn] = pn;


                TC.clear();
                F = vector<int>(numtarget, 0);
                for (int j = 0; j < A; j++)
                {
                    greedy_next_general_better(shift(TS), F);
                }


                int ts = differential_coverage(shift(TS), F);
                if (ts > sol_value)
                {
                    SOL = TS;
                    flag = true;
                    sol_value = ts;
                    break;
                }
            }
        }
        if (flag == false) break;
    }

    vector<int> FF = vector<int>(numtarget, 0);
    unordered_set<int> TT;
    TT.clear();
    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(SOL), FF);
    }
    return differential_coverage(shift(SOL), FF);
}


double obga, orga, lsa;


void animate(){
	//codes for any changes in Models, Camera




    int i, j, k, l;
	if (T % 10 == 0)
    {
        vector<robot> RC;
        for (i = 0; i < targetdata.size(); i++)
        {
            int cnt, gx, gy;
            unordered_set<int> st;
            st.clear();

            fprintf(fout, "%d %d\n", targetdata[i].x, targetdata[i].y);


            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (++gy < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                trajdata[i][0] = cell(gx, gy);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (--gy >= 0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                trajdata[i][1] = cell(gx, gy);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (++gx < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                trajdata[i][2] = cell(gx, gy);
            }

            gx = targetdata[i].x;
            gy = targetdata[i].y;
            cnt = 10;
            while (--gx >= 0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                trajdata[i][3] = cell(gx, gy);
            }

            robot rb;
            rb.cnt = 0;
            rb.ind = i;

            for (auto it = st.begin(); it != st.end(); it++) rb.cnt += 100 - G[*it / N][*it % N];

            RC.push_back(rb);
        }



        sort(RC.begin(), RC.end(), &comparator);




        int GC[N][N];
        for (i = 0; i < N; i++) for (j = 0; j < N; j++) GC[i][j] = G[i][j];


        for (i = 0; i < RC.size(); i++)
        {

            int ind = RC[i].ind;


            int cnt, gx, gy;
            unordered_set<int> st;



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (++gy < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int tt = 0;
            for (auto it = st.begin(); it != st.end(); it++) tt += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (--gy >=0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int bb = 0;
            for (auto it = st.begin(); it != st.end(); it++) bb += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (++gx < N && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int rr = 0;
            for (auto it = st.begin(); it != st.end(); it++) rr += 100 - GC[*it / N][*it % N];



            st.clear();
            gx = targetdata[ind].x;
            gy = targetdata[ind].y;
            cnt = 10;
            while (--gx >=0 && cnt-- && B[gx][gy] == true)
            {
                for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
            }

            int ll = 0;
            for (auto it = st.begin(); it != st.end(); it++) ll += 100 - GC[*it / N][*it % N];




            if (tt >= bb && tt >= ll && tt >= rr)
            {
                //cout << "t" << endl;
                targetdir[ind] = 1;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (++gy < N && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else if (bb >= tt && bb >= ll && bb >= rr)
            {
                //cout << "b" << endl;
                targetdir[ind] = 2;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (--gy >= 0 && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else if (rr >= tt && rr >= ll && rr >= bb)
            {
                //cout << "r" << endl;
                targetdir[ind] = 3;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (++gx < N && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }
            else
            {
                //cout << "l" << endl;
                targetdir[ind] = 4;
                st.clear();
                gx = targetdata[ind].x;
                gy = targetdata[ind].y;
                cnt = 10;
                while (--gx >= 0 && cnt-- && B[gx][gy] == true)
                {
                    for (j = 0; j < AL[gx * N + gy].size(); j++) st.insert(AL[gx * N + gy][j]);
                }
                for (auto it = st.begin(); it != st.end(); it++) GC[*it / N][*it % N] = 100;

            }

        }
        //cout << "ok" << endl;

        int op = old_paper();
        cout << op << endl;
        int obg = oblivious_greedy();
        cout << obg << endl;
        int org = ordered_greedy_cu();
        cout << org << endl;
        int ls = local_search_iog_atg();
        cout << ls << endl;
        cout << endl;



        obga += (double)obg / op;
        orga += (double)org / op;
        lsa += (double)ls / op;

        //fprintf(fout,"%d\n%d\n%d\n%d\n\n", op, obg, org, ls);

    }

    //write here








    //direction for each robot found above

//    for (i = 0; i < targetdir.size(); i++) cout << targetdir[i] << " ";
//    cout << endl;


    unordered_set<int> st;


    for (i = 0; i < targetdata.size(); i++)
    {
        if (targetdir[i] == 1)
        {
            if (targetdata[i].y < N - 1 && B[targetdata[i].x][targetdata[i].y + 1] == true)
                targetdata[i] = cell(targetdata[i].x, targetdata[i].y + 1);
        }
        else if (targetdir[i] == 2)
        {
            if (targetdata[i].y > 0 && B[targetdata[i].x][targetdata[i].y - 1] == true)
                targetdata[i] = cell(targetdata[i].x, targetdata[i].y - 1);
        }
        else if (targetdir[i] == 3)
        {
            if (targetdata[i].x < N - 1 && B[targetdata[i].x + 1][targetdata[i].y] == true)
                targetdata[i] = cell(targetdata[i].x + 1, targetdata[i].y);
        }
        else if (targetdir[i] == 4)
        {
            if (targetdata[i].x > 0 && B[targetdata[i].x - 1][targetdata[i].y] == true)
                targetdata[i] = cell(targetdata[i].x - 1, targetdata[i].y);
        }

        for (j = 0; j < AL[targetdata[i].x * N + targetdata[i].y].size(); j++)
                st.insert(AL[targetdata[i].x * N + targetdata[i].y][j]);
    }



    for (i = 0; i < N * N; i++)
    {
        auto it = st.find(i);
        if (it != st.end()) G[i / N][i % N] = 100;
        else if (G[i / N][i % N] > 0) G[i / N][i % N]--;
    }
    //cout << T << endl;

    T++;
}


//calculate visibility of cells around a robot
void processtarget(cell c)
{
    point p = c.c;
    vector<obs> vo = rangequery(p);
    int lgn = max(0, c.x - (int)(crange / cpx));
    int rgn = min(N - 1, c.x + (int)(crange / cpx));
    int bgn = max(0, c.y - (int)(crange / cpx));
    int tgn = min(N - 1, c.y + (int)(crange / cpx));

    int i, j, k;
    for (i = lgn; i <= rgn; i++)
    {
        for (j = bgn; j <= tgn; j++)
        {
            point tp = point((i + 0.5) * cpx, (j + 0.5) * cpx);
            if (B[i][j] == false) continue;
            if ((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y) > crange * crange) continue;
            bool flag = true;
            for (k = 0; k < vo.size(); k++)
            {
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xl, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xh, vo[k].yl), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xh, vo[k].yl)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yh), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
            }
            if (flag == true)
            {
                G[i][j] = 100;
            }
        }
    }
}

//find the cells visible from a given cell
vector<int> processcell(cell c)
{
    point p = c.c;
    vector<int> ret;
    ret.clear();
    vector<obs> vo = rangequery(p);
    int lgn = max(0, c.x - (int)(crange / cpx));
    int rgn = min(N - 1, c.x + (int)(crange / cpx));
    int bgn = max(0, c.y - (int)(crange / cpx));
    int tgn = min(N - 1, c.y + (int)(crange / cpx));

    int i, j, k;
    for (i = lgn; i <= rgn; i++)
    {
        for (j = bgn; j <= tgn; j++)
        {
            point tp = point((i + 0.5) * cpx, (j + 0.5) * cpx);
            if (B[i][j] == false) continue;
            if ((p.x - tp.x) * (p.x - tp.x) + (p.y - tp.y) * (p.y - tp.y) > crange * crange) continue;
            bool flag = true;
            for (k = 0; k < vo.size(); k++)
            {
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xl, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xh, vo[k].yl), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yl), point(vo[k].xh, vo[k].yl)))
                {
                    flag = false;
                    break;
                }
                if (doesint(p, tp, point(vo[k].xl, vo[k].yh), point(vo[k].xh, vo[k].yh)))
                {
                    flag = false;
                    break;
                }
            }
            if (flag == true)
            {
                ret.push_back(i * N + j);
            }
        }
    }
    return ret;
}



int main(){
    T = 0;
    int i, j, k;

    fout = fopen("traj.txt", "w");
    fin = fopen("obs.txt", "r");

	//genobs();

	int tno;
	fscanf(fin, " %d", &tno);
	obsdata.clear();
	for (i = 0; i < tno; i++)
    {
        obs to;
        fscanf(fin, " %lf %lf %lf %lf", &to.xl, &to.xh, &to.yl, &to.yh);
        cout << to.xl << " " << to.xh << " " << to.yl << " " << to.yh << endl;
        obsdata.push_back(to);
    }

	genblockedcells();
    gentarget();
    gentraj();

    obga = 0;
    orga = 0;
    lsa = 0;



    for (i = 0; i < N; i++) for (j = 0; j < N; j++) G[i][j] = 0;

    for (i = 0; i < targetdata.size(); i++) processtarget(targetdata[i]);

    AL.clear();
    for (i = 0; i < N; i++) for (j = 0; j < N; j++) AL.push_back(processcell(cell(i, j)));

    while (T <= 1000) animate();

    cout << obga/21.0 << endl;
    cout << orga/21.0 << endl;
    cout << lsa/21.0 << endl;

    fclose(fout);


	return 0;
}

