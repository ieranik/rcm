#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <sys/time.h>
#include <unordered_set>
#include <map>

using namespace std;

#define R 1000
#define PpR 3
#define P (R * PpR)
#define T 1000
#define TpP 10
#define SD 5
#define A 10
#define N 10
#define TF true
#define tlen 1000
#define trad 100
#define pi 3.14159265359


vector< vector<int> > TL;
vector< vector<int> > G;
vector<int> OPT_S;
vector<int> OPT_A;

vector<int> OLD_S;
vector<int> OLD_A;

int rr = 0;

class point
{
public:
    int x, y;
    point(int xx = 0, int yy = 0)
    {
        x = xx;
        y = yy;
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

vector<int> remove_duplicate(vector<int>& V)
{
    vector<int> F = vector<int>(T, 0);
    vector<int> RT;
    RT.clear();
    for (int i = 0; i < V.size(); i++) if (F[V[i]] == 0) F[V[i]] = 1;
    for (int i = 0; i < T; i++) if (F[i] == 1) RT.push_back(i);
    return RT;
}

void construct_random()
{
    srand(time(NULL) + rr++);

    TL = vector< vector<int> > (P, vector<int>());

    for (int i = 0; i < P; i++)
    {
        int num_targets = rand() % (2 * SD + 1) + TpP - SD;
        while (num_targets--) TL[i].push_back(rand() % T);
        TL[i] = remove_duplicate(TL[i]);
    }
}

double sqr_dis(point a, point b)
{
    return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

void construct_geometric()
{
    srand(time(NULL) + rr++);

    TL = vector< vector<int> > (P, vector<int>());

    vector<point> TP;
    TP.clear();

    vector<point> RP;
    RP.clear();

    for (int i = 0; i < T; i++)
    {
        point tp;
        tp.x = rand() % 10000;
        tp.y = rand() % 10000;
        TP.push_back(tp);
    }

    for (int i = 0; i < R; i++)
    {
        point rp;
        rp.x = rand() % 10000;
        rp.y = rand() % 10000;
        RP.push_back(rp);
    }

    vector<double> tvals;
    tvals.clear();

    for (int j = 0; j < PpR; j++)
    {
        double theta = (-90.0 + (180.0 / (PpR + 1)) * (j + 1)) * (pi / 180.0);
        double X = tlen * cos(theta);
        double Y = tlen * sin(theta);
        double clen = 0.0;
        double tpar = 270.0;
        while (clen < tlen)
        {
            point prev;
            prev.x = X * cos((tpar) * (pi /  180.0));
            prev.y = Y * sin((tpar) * (pi /  180.0));

            tpar += 1.0;

            point pnew;
            pnew.x = X * cos((tpar) * (pi /  180.0));
            pnew.y = Y * sin((tpar) * (pi /  180.0));

            clen += sqrt(sqr_dis(prev, pnew));
        }
        tvals.push_back((tpar - 271.0) / 9.0);
    }

    //for (int i = 0; i < tvals.size(); i++) cout << tvals[i] << endl;


    for (int i = 0; i < R; i++)
    {
        int rx = RP[i].x;
        int ry = RP[i].y;

        for (int j = 0; j < PpR; j++)
        {
            double theta = (-90.0 + (180.0 / (PpR + 1)) * (j + 1)) * (pi / 180.0);
            double X = tlen * cos(theta);
            double Y = tlen * sin(theta);

            vector<point> tps;
            tps.clear();

            for (int k = 0; k < 10; k++)
            {
                point npt;
                npt.x = rx + X * cos((270.0 + k * tvals[j]) * (pi /  180.0));
                npt.y = ry + Y * (1 + sin((270.0 + k * tvals[j]) * (pi /  180.0)));
                tps.push_back(npt);
            }

            for (int k = 0; k < T; k++)
            {
                for (int l = 0; l < (int)tps.size(); l++)
                {
                    if (sqr_dis(tps[l], TP[k]) < trad * trad)
                    {
                        TL[i * PpR + j].push_back(k);
                        break;
                    }
                }
            }
        }
    }
}


vector<int> shift(vector<int> V)
{
    for (int i = 1; i < V.size(); i++) V[i] += i * PpR;
    return V;
}

void print_V(vector<int> V)
{
    for (int i = 0; i < V.size(); i++)
        cout << V[i] << "\t";
    cout << endl << endl;
}

void print_V2(vector< vector<int> > TL)
{
    for (int i = 0; i < TL.size(); i++)
    {
        cout << i << "\t" << TL[i].size() << "\t";
        for (int j = 0; j < TL[i].size(); j++)
            cout << TL[i][j] << "\t";
        cout << endl;
    }
    cout << endl;
}

int calculate_coverage(vector<int> S)
{
    vector<int> V = vector<int> (T, 0);
    for (int i = 0; i < S.size(); i++)
        for (int j = 0; j < TL[S[i]].size(); j++) V[TL[S[i]][j]] = 1;

    int coverage = 0;
    for (int i = 0; i < V.size(); i++) if (V[i] == 1) coverage++;

    return coverage;
}

int count_bits(int num)
{
    int ret = 0;
    while (num != 0)
    {
        if (num & 1) ret++;
        num /= 2;
    }
    return ret;
}

vector<int> next_combination(vector<int> S)
{
    int num = 0;
    vector<int> _S = vector<int>(S.size(), 0);
    for (int i = 0; i < S.size(); i++) if (S[i] == 1) num += (1 << i);
    if (num == ((1 << A) - 1) << (R - A))
    {
        _S[0] = -1;
        return _S;
    }
    while (true)
    {
        num++;
        if (count_bits(num) == A) break;
    }
    for (int i = 0; i < S.size(); i++) if (num & (1 << i)) _S[i] = 1;
    return _S;
}

vector<int> next_selection(vector<int> S)
{
    for (int i = 0; i < S.size(); i++)
    {
        if (i == S.size() - 1 && S[i] == PpR - 1)
        {
            S[0] = -1;
            return S;
        }
        if (S[i] == PpR - 1)
        {
            S[i] = 0;
            continue;
        }
        if (S[i] < PpR - 1)
        {
            S[i]++;
            return S;
        }
    }
}


//calculate the coverage of the robots not in D
int differential_coverage(vector<int> S, vector<int> D)
{
    vector<int> V = vector<int> (T, 0);
    for (int i = 0; i < S.size(); i++)
    {
        if (D[i] == 0)
        {
            for (int j = 0; j < TL[S[i]].size(); j++) V[TL[S[i]][j]] = 1;
        }
    }


    int coverage = 0;
    for (int i = 0; i < V.size(); i++) if (V[i] == 1) coverage++;

    return coverage;
}


//return the optimum attack of size A for which differential coverage is minimum
vector<int> min_diff_coverage(vector<int> S)
{
    vector<int> D = vector<int>(S.size(), 0);
    for (int i = 0; i < A; i++) D[i] = 1;
    vector<int> ret = D;
    int min_coverage = differential_coverage(S, D);
    while (true)
    {
        D = next_combination(D);
        if (D[0] == -1) break;
        int coverage = differential_coverage(S, D);
        if (coverage < min_coverage)
        {
            min_coverage = coverage;
            ret = D;
        }
    }
    return ret;
}

//calculates optimum solution in OPT_S and OPT_A
void optimum()
{
    vector<int> S = vector<int> (R, 0);
    OPT_S = shift(S);
    vector<int> D = min_diff_coverage(shift(S));
    OPT_A = D;
    int max_coverage = differential_coverage(shift(S), D);
    while (true)
    {
        S = next_selection(S);
        if (S[0] == -1) break;

        D = min_diff_coverage(shift(S));


        int coverage = differential_coverage(shift(S), D);

        //cout << coverage << endl;

        if (coverage > max_coverage)
        {
            max_coverage = coverage;
            OPT_S = shift(S);
            OPT_A = D;
        }
    }
}

//select from robots with S[]=-1 for which incremental coverage is maximized
void greedy_next(vector<int>& S, vector<int>& TC)
{
    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < R; i++)
    {
        if (S[i] == -1)
        {
            for (int j = 0; j < PpR; j++)
            {
                int cnt = 0;
                for (int k = 0; k < TL[i * PpR + j].size(); k++)
                {
                    if (TC[TL[i * PpR + j][k]] == 0) cnt++;
                }
                if (cnt > max_coverage)
                {
                    max_coverage = cnt;
                    max_index = i * PpR + j;
                }
            }
        }
    }
    S[max_index / PpR] = max_index % PpR;
    for (int i = 0; i < TL[max_index].size(); i++) TC[TL[max_index][i]] = 1;
}

//old paper implementation
int old_paper()
{
    vector<int> S1 = vector<int>(R, -1);
    for (int i = 0; i < R; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < PpR; j++)
        {
            if ((int)TL[i * PpR + j].size() > max_coverage)
            {
                max_coverage = TL[i * PpR + j].size();
                max_index = j;
            }
        }
        S1[i] = max_index;
    }

    //print_V(S1);

    vector<int> V = vector<int>(R, 0);
    for (int i = 0; i < A; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < R; j++)
        {
            if (V[j] == 0)
            {
                if ((int)TL[S1[j] + j * PpR].size() > max_coverage)
                {
                    max_coverage = TL[S1[j] + j * PpR].size();
                    max_index = j;
                }
            }
        }
        V[max_index] = 1;
    }
    //print_V(V);

    vector<int> S2 = vector<int>(R, -1);
    vector<int> TC = vector<int>(T, 0);
    for (int i = 0; i < R; i++)
    {
        if (V[i] == 1)
        {
            S2[i] = S1[i];
            //for (int j = 0; j < TL[i * PpR + S1[i]].size(); j++) TC[TL[i * PpR + S1[i]][j]] = 1;
        }
    }

    for (int i = A; i < R; i++)
    {
        //print_V(S2);
        greedy_next(S2, TC);
    }
    //print_V(S2);

    //cout << differential_coverage(shift(S2), V) << endl;


    if (!TF)
    {
        vector<int> D = min_diff_coverage(shift(S2));
        return differential_coverage(shift(S2), D);
    }

    return 0;

}

//for each robot, select the path with highest targets
int check()
{
    vector<int> S1 = vector<int>(R, -1);
    for (int i = 0; i < R; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < PpR; j++)
        {
            if ((int)TL[i * PpR + j].size() > max_coverage)
            {
                max_coverage = TL[i * PpR + j].size();
                max_index = j;
            }
        }
        S1[i] = max_index;
    }

    if (!TF)
    {
        vector<int> D = min_diff_coverage(shift(S1));
        return differential_coverage(shift(S1), D);
    }

    return 0;
}


vector<int> robot_permutation(vector<int> S)
{
    vector<int> TC = vector<int>(T, 0);
    vector<int> SQ = vector<int>(R, 0);

    //print_V(S);

    for (int i = 0; i < S.size(); i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < PpR; j++)
        {
            int cnt = 0;
            for (int k = 0; k < TL[S[i] * PpR + j].size(); k++)
            {
                if (TC[TL[S[i] * PpR + j][k]] == 0) cnt++;
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = j;

            }
            //cout << j << " " << cnt << endl;
        }
        SQ[S[i]] = max_index;
        //cout << S[i] << " " << max_index << endl << endl;
        for (int j = 0; j < TL[S[i] * PpR + max_index].size(); j++) TC[TL[S[i] * PpR + max_index][j]] = 1;
    }
    return SQ;
}

int check2()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        int cnt = 0;
        for (int j = 0; j < PpR; j++)
        {
            cnt += TL[i * PpR + j].size();
        }
        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &comparator);

//    for (int i = 0; i < RC.size(); i++) cout << RC[i].ind << " " << RC[i].cnt << endl;
//
//    cout << endl;

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    if (!TF)
    {
        vector<int> D = min_diff_coverage(shift(RES));
        int res = differential_coverage(shift(RES), D);
        return res;
    }

    return 0;
}


//select the robot not in F which has the maximum number of targets which none of the other robots covers
void greedy_next_general_better(vector<int> S, vector<int>& F)
{
    vector<int> C = vector<int>(T, 0);
    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            for (int j = 0; j < TL[S[i]].size(); j++)
                C[TL[S[i]][j]]++;
        }
    }
    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            int cnt = 0;
            for (int j = 0; j < TL[S[i]].size(); j++)
            {
                if (C[TL[S[i]][j]] == 1) cnt++;
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = i;
            }
        }
    }
    F[max_index] = 1;
}

//select next robot not in F such that incremental coverage is maximized
void greedy_next_general(vector<int> S, vector<int>& F, vector<int>& TC)
{
    int max_coverage = -1;
    int max_index = -1;
    for (int i = 0; i < S.size(); i++)
    {
        if (F[i] == 0)
        {
            int cnt = 0;
            for (int k = 0; k < TL[S[i]].size(); k++)
            {
                if (TC[TL[S[i]][k]] == 0) cnt++;
            }
            if (cnt > max_coverage)
            {
                max_coverage = cnt;
                max_index = i;
            }
        }
    }
    F[max_index] = 1;
    for (int i = 0; i < TL[max_index].size(); i++) TC[TL[max_index][i]] = 1;
}


void addp(vector<unordered_set<int> > &tset, unordered_set<int> &ones, unordered_set<int> &zeros, int p)
{
    for (int i = 0; i < TL[p].size(); i++)
    {
        if ((int)tset[TL[p][i]].size() == 0)
        {
            ones.insert(TL[p][i]);
            zeros.erase(TL[p][i]);
        }
        if ((int)tset[TL[p][i]].size() == 1)
        {
            ones.erase(TL[p][i]);
        }
        tset[TL[p][i]].insert(p);
    }
}

void delp(vector<unordered_set<int> > &tset, unordered_set<int> &ones, unordered_set<int> &zeros, int p)
{
    for (int i = 0; i < TL[p].size(); i++)
    {
        if ((int)tset[TL[p][i]].size() == 1)
        {
            ones.erase(TL[p][i]);
            zeros.insert(TL[p][i]);
        }
        if ((int)tset[TL[p][i]].size() == 2)
        {
            ones.insert(TL[p][i]);
        }
        tset[TL[p][i]].erase(p);
    }
}

int local_search()
{
    vector<int> S1 = vector<int>(R, -1);
    for (int i = 0; i < R; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < PpR; j++)
        {
            if ((int)TL[i * PpR + j].size() > max_coverage)
            {
                max_coverage = TL[i * PpR + j].size();
                max_index = j;
            }
        }
        S1[i] = max_index;
    }
    //S1 = shift(S1);

    //print_V(shift(S1));



    vector<int> F = vector<int>(R, 0);
    //vector<int> TC = vector<int>(T, 0);
    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(S1), F);
    }

    vector<int> SOL = S1;
    int sol_value = differential_coverage(shift(S1), F);


    vector<unordered_set<int> > tset = vector<unordered_set<int> >(T, unordered_set<int>());
    vector<int> SS = shift(S1);
    for (int i = 0; i < SS.size(); i++)
    {
        for (int j = 0; j < TL[SS[i]].size(); j++)
            tset[TL[SS[i]][j]].insert(SS[i]);
    }
    unordered_set<int> ones;
    unordered_set<int> zeros;

    ones.clear();
    zeros.clear();

    for (int i = 0; i < T; i++)
    {
        if ((int)tset[i].size() == 0) zeros.insert(i);
        if ((int)tset[i].size() == 1) ones.insert(i);
    }


    while (true)
    {
        bool flag = false;
        for (int i = 0; i < R * PpR; i++)
        {
            int rn = i / PpR;
            int pn = i % PpR;
            if (SOL[rn] != pn)
            {
                vector<int> TS = SOL;
                TS[rn] = pn;

                int newrp = i;
                int oldrp = rn * PpR + SOL[rn];

                addp(tset, ones, zeros, newrp);
                delp(tset, ones, zeros, oldrp);

                vector<int> ch = vector<int>();

                for (int j = 0; j < A; j++)
                {
                    map<int, int> mp = map<int, int>();
                    for (auto it = ones.begin(); it != ones.end(); it++)
                    {
                        int tarn = *it;
                        int rpn = *tset[tarn].begin();
                        if (mp.find(rpn) == mp.end()) mp[rpn] = 1;
                        else mp[rpn]++;
                    }
                    int mx = -1;
                    int mxidx = j * PpR;
                    for (auto it = mp.begin(); it != mp.end(); it++)
                    {
                        pair<int, int> pii = *it;
                        if (mx < pii.second)
                        {
                            mx = pii.second;
                            mxidx = pii.first;
                        }
                    }
                    ch.push_back(mxidx);
                    delp(tset, ones, zeros, mxidx);

                    //greedy_next_general_better(shift(TS), F);
                }


                int ts = T - zeros.size();
                if (ts > sol_value)
                {
                    SOL = TS;
                    sol_value = ts;
                    flag = true;
                    break;
                }

                for (int j = 0; j < ch.size(); j++) addp(tset, ones, zeros, ch[j]);
                addp(tset, ones, zeros, oldrp);
                delp(tset, ones, zeros, newrp);



            }
        }
        if (flag == false)
            break;
    }

    if(!TF)
    {
        vector<int> D = min_diff_coverage(shift(SOL));
        return differential_coverage(shift(SOL), D);
    }

    return 0;
}




//attacker uses better greedy
int local_search2()
{
    vector<int> S1 = vector<int>(R, -1);
    for (int i = 0; i < R; i++)
    {
        int max_coverage = -1;
        int max_index = -1;
        for (int j = 0; j < PpR; j++)
        {
            if ((int)TL[i * PpR + j].size() > max_coverage)
            {
                max_coverage = TL[i * PpR + j].size();
                max_index = j;
            }
        }
        S1[i] = max_index;
    }
    //S1 = shift(S1);

    //print_V(shift(S1));



    vector<int> F = vector<int>(R, 0);
    //vector<int> TC = vector<int>(T, 0);
    for (int i = 0; i < A; i++)
    {
        greedy_next_general_better(shift(S1), F);
    }

    vector<int> SOL = S1;
    int sol_value = differential_coverage(shift(S1), F);

    while (true)
    {
        bool flag = false;
        for (int i = 0; i < R * PpR; i++)
        {
            int rn = i / PpR;
            int pn = i % PpR;
            if (SOL[rn] != pn)
            {
                vector<int> TS = SOL;
                TS[rn] = pn;

                F = vector<int>(R, 0);
                for (int j = 0; j < A; j++)
                {
                    greedy_next_general_better(shift(TS), F);
                }


                int ts = differential_coverage(shift(TS), F);
                if (ts > sol_value)
                {
                    SOL = TS;
                    sol_value = ts;
                    flag = true;
                    break;
                }
            }
        }
        if (flag == false)
            break;
    }

    if(!TF)
    {
        vector<int> D = min_diff_coverage(shift(SOL));
        return differential_coverage(shift(SOL), D);
    }

    return 0;
}


int main()
{
    int num = N;
    int opt_average = 0;
    int old_average = 0;
    int ls_average = 0;
    int nv_average = 0;
    int lnr_average = 0;

    long int oldt = 0;
    long int lst = 0;
    long int nvt = 0;
    long int lnrt = 0;

    struct timeval tps;
    struct timeval tpe;

    long int st;
    long int et;


    while (num--)
    {
        //construct_geometric();
        construct_random();

        if (!TF)
        {
            optimum();
            //cout << "Optimum:" << endl;
            int opt = differential_coverage(OPT_S, OPT_A);
            opt_average += opt;
            cout << opt << endl;
        }

        //cout << "Old Paper:" << endl;
        gettimeofday(&tps, NULL);
        st = tps.tv_sec * 1000000 + tps.tv_usec ;

        int old = old_paper();

        gettimeofday(&tpe, NULL);
        et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

        oldt += (et - st);
        old_average += old;
        cout << old << endl;



        //cout << "Idea 1:" << endl;
        gettimeofday(&tps, NULL);
        st = tps.tv_sec * 1000000 + tps.tv_usec ;

        int ls = local_search();

        gettimeofday(&tpe, NULL);
        et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

        lst += (et - st);
        ls_average += ls;
        cout << ls << endl;



        //cout << "Check:" << endl;
        gettimeofday(&tps, NULL);
        st = tps.tv_sec * 1000000 + tps.tv_usec ;

        int nv = check();

        gettimeofday(&tpe, NULL);
        et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

        nvt += (et - st);
        nv_average += nv;
        cout << nv << endl;



        //cout << "Check:" << endl;
        gettimeofday(&tps, NULL);
        st = tps.tv_sec * 1000000 + tps.tv_usec ;

        int lnr = check2();

        gettimeofday(&tpe, NULL);
        et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

        lnrt += (et - st);
        lnr_average += lnr;
        cout << lnr << endl;


        cout << endl;
    }

    if (!TF)
    {
        cout << "Brute Force Optimum:" << endl;
        cout << (double)opt_average / N << endl << endl;
    }

    cout << "Previous Work:" << endl;
    cout << (double)old_average / N << " " << (double)old_average / (double)opt_average << endl << endl;

    cout << "Local Search:" << endl;
    cout << (double)ls_average / N << " " << (double)ls_average / (double)opt_average << endl << endl;

    cout << "Naive:" << endl;
    cout << (double)nv_average / N << " " << (double)nv_average / (double)opt_average << endl << endl;

    cout << "Linear Greedy:" << endl;
    cout << (double)lnr_average / N << " " << (double)lnr_average / (double)opt_average << endl << endl;


    cout << "old: " << oldt << endl;
    cout << "ls: " << lst << endl;
    cout << "nv: " << nvt << endl;
    cout << "org: " << lnrt << endl;


    return 0;
}

