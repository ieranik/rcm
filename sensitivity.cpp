#include <iostream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <algorithm>
#include <sys/time.h>
#include <unordered_set>
#include <map>
#include <fstream>


using namespace std;

#define R 15
#define PpR 4
#define P (R * PpR)
#define TpP 10
#define SD 5
#define N 50
#define T 150
#define A 6
#define TF false


vector< vector<int> > TL;
vector< vector<int> > G;
vector<int> OPT_S;
vector<int> OPT_A;

vector<int> OLD_S;
vector<int> OLD_A;

int rr = 0;
int sns;


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

bool rcomparator(const robot& lhs, const robot& rhs)
{
   return lhs.cnt > rhs.cnt;
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

    for (int i = 0; i < R; i++)
    {
        int rx = RP[i].x;
        int ry = RP[i].y;

        int xl, xh, yl, yh;

        xl = rx - 500;
        xh = rx + 500;
        yl = ry - 500;
        yh = ry + 3500;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 0].push_back(j);
        }

        xl = rx - 500;
        xh = rx + 500;
        yl = ry - 3500;
        yh = ry + 500;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 1].push_back(j);
        }

        xl = rx - 500;
        xh = rx + 3500;
        yl = ry - 500;
        yh = ry + 500;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 2].push_back(j);
        }

        xl = rx - 3500;
        xh = rx + 500;
        yl = ry - 500;
        yh = ry + 500;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 3].push_back(j);
        }
    }
}

double cal_density()
{
    vector<int> den = vector<int>(T, 0);
    for (int i = 0; i < TL.size(); i++) for (int j = 0; j < TL[i].size(); j++) den[TL[i][j]]++;
    int cnt = 0;
    int sum = 0;
    for (int i = 0; i < den.size(); i++)
    {
        if (den[i] != 0) cnt++;
        sum += den[i];
    }
    return (double)sum/cnt;
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
    for (int i = 0; i < sns; i++) D[i] = 1;
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


    vector<int> D = min_diff_coverage(shift(S2));


    int coverage = differential_coverage(shift(S2), D);

    return coverage;
}


//oblivious Greedy
//for each robot, select the path with highest targets
int oblivious_greedy()
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

    vector<int> D = min_diff_coverage(shift(S1));


    int coverage = differential_coverage(shift(S1), D);

    return coverage;
}


//given an ordering of the robots, make R greedy choices to maximize incremental target coverage
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


//ordered greedy - robots are ordered by increasing cardinality of union of target coverage
int ordered_greedy_cu()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        vector<int> U = vector<int>(T, 0);
        for (int j = 0; j < PpR; j++)
        {
            for (int k = 0; k < TL[i * PpR + j].size(); k++)
                U[TL[i * PpR + j][k]] = 1;
        }
        int cnt = 0;
        for (int j = 0; j < U.size(); j++) if (U[j] == 1) cnt++;

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

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

}


//ordered greedy - robots are ordered by increasing sum of target coverages of all trajectories
int ordered_greedy_sc()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        int cnt = 0;
        for (int j = 0; j < PpR; j++) cnt += TL[i * PpR + j].size();

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

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

}

//ordered greedy - robots are ordered by increasing max target coverage of single trajectory
int ordered_greedy_mt()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        int mx = -1;
        for (int j = 0; j < PpR; j++)
        {
            if (mx < (int)TL[i * PpR + j].size()) mx = (int)TL[i * PpR + j].size();
        }

        robot rb;
        rb.cnt = mx;
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

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

}



//ordered greedy - robots are ordered by increasing cardinality of union of target coverage
int ordered_greedy_cur()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        vector<int> U = vector<int>(T, 0);
        for (int j = 0; j < PpR; j++)
        {
            for (int k = 0; k < TL[i * PpR + j].size(); k++)
                U[TL[i * PpR + j][k]] = 1;
        }
        int cnt = 0;
        for (int j = 0; j < U.size(); j++) if (U[j] == 1) cnt++;

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &rcomparator);

//    for (int i = 0; i < RC.size(); i++) cout << RC[i].ind << " " << RC[i].cnt << endl;
//
//    cout << endl;

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

}



//ordered greedy - robots are ordered by increasing sum of target coverages of all trajectories
int ordered_greedy_scr()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        int cnt = 0;
        for (int j = 0; j < PpR; j++) cnt += TL[i * PpR + j].size();

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &rcomparator);

//    for (int i = 0; i < RC.size(); i++) cout << RC[i].ind << " " << RC[i].cnt << endl;
//
//    cout << endl;

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

}

//ordered greedy - robots are ordered by increasing max target coverage of single trajectory
int ordered_greedy_mtr()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        int mx = -1;
        for (int j = 0; j < PpR; j++)
        {
            if (mx < (int)TL[i * PpR + j].size()) mx = (int)TL[i * PpR + j].size();
        }

        robot rb;
        rb.cnt = mx;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &rcomparator);

//    for (int i = 0; i < RC.size(); i++) cout << RC[i].ind << " " << RC[i].cnt << endl;
//
//    cout << endl;

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    vector<int> D = min_diff_coverage(shift(RES));


    int coverage = differential_coverage(shift(RES), D);


    return coverage;

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



//initialized using ordered greedy, attacker uses traditional greedy
int local_search_iog_atg()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        vector<int> U = vector<int>(T, 0);
        for (int j = 0; j < PpR; j++)
        {
            for (int k = 0; k < TL[i * PpR + j].size(); k++)
                U[TL[i * PpR + j][k]] = 1;
        }
        int cnt = 0;
        for (int j = 0; j < U.size(); j++) if (U[j] == 1) cnt++;

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);

    }

    sort(RC.begin(), RC.end(), &comparator);

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    vector<int> S1 = RES;


    vector<int> TC = vector<int>(T, 0);
    vector<int> F = vector<int>(R, 0);
    for (int i = 0; i < A; i++)
    {
        greedy_next_general(shift(S1), F, TC);
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

                //F = min_diff_coverage(shift(TS));


                TC = vector<int>(T, 0);
                F = vector<int>(R, 0);
                for (int j = 0; j < A; j++)
                {
                    greedy_next_general(shift(TS), F, TC);
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

    vector<int> D = min_diff_coverage(shift(SOL));


    int coverage = differential_coverage(shift(SOL), D);


    return coverage;


}


//initialized using oblivious greedy, attacker uses traditional greedy
int local_search_inv_atg()
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


    vector<int> TC = vector<int>(T, 0);
    vector<int> F = vector<int>(R, 0);
    for (int i = 0; i < A; i++)
    {
        greedy_next_general(shift(S1), F, TC);
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

                //F = min_diff_coverage(shift(TS));


                TC = vector<int>(T, 0);
                F = vector<int>(R, 0);
                for (int j = 0; j < A; j++)
                {
                    greedy_next_general(shift(TS), F, TC);
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

    vector<int> D = min_diff_coverage(shift(SOL));


    int coverage = differential_coverage(shift(SOL), D);


    return coverage;


}



//initialized using oblivious greedy, attacker uses better greedy
int local_search_inv_abg()
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

    vector<int> D = min_diff_coverage(shift(SOL));


    int coverage = differential_coverage(shift(SOL), D);


    return coverage;
}


//initialized using ordered greedy, attacker uses better greedy
int local_search_iog_abg()
{
    vector<robot> RC;
    for (int i = 0; i < R; i++)
    {
        vector<int> U = vector<int>(T, 0);
        for (int j = 0; j < PpR; j++)
        {
            for (int k = 0; k < TL[i * PpR + j].size(); k++)
                U[TL[i * PpR + j][k]] = 1;
        }
        int cnt = 0;
        for (int j = 0; j < U.size(); j++) if (U[j] == 1) cnt++;

        robot rb;
        rb.cnt = cnt;
        rb.ind = i;
        RC.push_back(rb);
    }

    sort(RC.begin(), RC.end(), &comparator);

    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = RC[i].ind;

    RES = robot_permutation(PR);

    vector<int> S1 = RES;




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

    vector<int> D = min_diff_coverage(shift(SOL));


    int coverage = differential_coverage(shift(SOL), D);


    return coverage;
}

int main()
{
    int num = N;

    ofstream myfile("data_4_geometric.txt");


    for (sns = 0; sns < 13; sns += 2)
    {
        int opt_average = 0;

        int old_average = 0;
        int nv_average = 0;
        double nv_max = -1.0;
        double nv_min = 1000.0;
        vector<double> nv_arr;

        int lsot_average = 0;
        double lsot_max = -1.0;
        double lsot_min = 1000.0;
        vector<double> lsot_arr;
        int lsob_average = 0;
        double lsob_max = -1.0;
        double lsob_min = 1000.0;
        vector<double> lsob_arr;
        int lsnt_average = 0;
        double lsnt_max = -1.0;
        double lsnt_min = 1000.0;
        vector<double> lsnt_arr;
        int lsnb_average = 0;
        double lsnb_max = -1.0;
        double lsnb_min = 1000.0;
        vector<double> lsnb_arr;

        int cu_average = 0;
        double cu_max = -1.0;
        double cu_min = 1000.0;
        vector<double> cu_arr;
        int cur_average = 0;
        double cur_max = -1.0;
        double cur_min = 1000.0;
        vector<double> cur_arr;
        int sc_average = 0;
        double sc_max = -1.0;
        double sc_min = 1000.0;
        vector<double> sc_arr;
        int scr_average = 0;
        double scr_max = -1.0;
        double scr_min = 1000.0;
        vector<double> scr_arr;
        int mt_average = 0;
        double mt_max = -1.0;
        double mt_min = 1000.0;
        vector<double> mt_arr;
        int mtr_average = 0;
        double mtr_max = -1.0;
        double mtr_min = 1000.0;
        vector<double> mtr_arr;



        long int oldt = 0;
        long int nvt = 0;

        long int lsobt = 0;
        long int lsott = 0;
        long int lsnbt = 0;
        long int lsntt = 0;

        long int cut = 0;
        long int curt = 0;
        long int sct = 0;
        long int scrt = 0;
        long int mtt = 0;
        long int mtrt = 0;


        struct timeval tps;
        struct timeval tpe;

        long int st;
        long int et;

        num = N;
        myfile << sns << "\t";

        while (num--)
        {

            //T = rand() % 540 + 60;
            //construct_random();
            //cout << cal_density() << endl;


            //param = rand() % 10000 + 5000;
            construct_geometric();

            //cout << cal_density() << endl;




            //Old Paper
            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int old = old_paper();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            oldt += (et - st);
            old_average += old;
            cout << old << endl;



            //Oblivious Greedy
            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int nv = oblivious_greedy();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            nvt += (et - st);
            nv_average += nv;
            nv_arr.push_back(100.0*(double)nv/old);
            if ((double)nv/old > nv_max) nv_max = (double)nv/old;
            if ((double)nv/old < nv_min) nv_min = (double)nv/old;

            cout << nv << endl;



            //Local Search Algorithms
            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int lsob = local_search_iog_abg();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            lsobt += (et - st);
            lsob_average += lsob;
            lsob_arr.push_back(100.0*(double)lsob/old);
            if ((double)lsob/old > lsob_max) lsob_max = (double)lsob/old;
            if ((double)lsob/old < lsob_min) lsob_min = (double)lsob/old;
            cout << lsob << endl;



            //Ordered Greedy Algorithms
            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int cu = ordered_greedy_cu();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            cut += (et - st);
            cu_average += cu;
            cu_arr.push_back(100.0*(double)cu/old);
            if ((double)cu/old > cu_max) cu_max = (double)cu/old;
            if ((double)cu/old < cu_min) cu_min = (double)cu/old;
            cout << cu << endl;


            cout << endl;

//            myfile << "100.00" << "\t";
//
//            myfile << 100.0*(double)nv/old << "\t";
//            myfile << 100.0*(double)lsob/old << "\t";
//            myfile << 100.0*(double)cu/old << "\n";
        }

        cout << "Old Algorithm" << endl;
        cout << "old:\t" << old_average / (double)N << endl;
        myfile << 100.0 * old_average / old_average << "\t";

        cout << "Oblivious Greedy Algorithm" << endl;
        cout << "obg:\t" << nv_average / (double)N << endl;
        myfile << 100.0 * nv_average / old_average << "\t";
//        sort(nv_arr.begin(), nv_arr.end());
//        myfile << nv_arr[74] << "\t";
//        myfile << nv_arr[24] << "\t";

        cout << "Local Searchs" << endl;
        cout << "lsob:\t" << lsob_average / (double)N << endl;
        myfile << 100.0 * lsob_average / old_average << "\t";
//        sort(lsob_arr.begin(), lsob_arr.end());
//        myfile << lsob_arr[74] << "\t";
//        myfile << lsob_arr[24] << "\t";



        cout << "Ordered Greedy Algorithms" << endl;
        cout << "cu:\t" << cu_average / (double)N << endl;
        myfile << 100.0 * cu_average / old_average << "\t\n";
//        sort(cu_arr.begin(), cu_arr.end());
//        myfile << cu_arr[74] << "\t";
//        myfile << cu_arr[24] << "\t";



    }

    myfile.close();
    return 0;
}


/*

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


int idea1()
{
    int max_coverage = -1;
    int num = 100;
    vector<int> PR = vector<int>(R, 0);
    vector<int> RES;
    for (int i = 0; i < R; i++) PR[i] = i;
    //do
    {
        random_shuffle(PR.begin(), PR.end());
        RES = robot_permutation(PR);

        vector<int> D = min_diff_coverage(shift(RES));
        int res = differential_coverage(shift(RES), D);
        if (res > max_coverage) max_coverage = res;
        //print_V(PR);
    }
    //while (next_permutation(PR.begin(), PR.end()));
    return max_coverage;
}

bool intersect(int p, int q)
{
    vector<int> V = vector<int> (T, 0);
    for (int i = 0; i < TL[p].size(); i++) V[TL[p][i]] = 1;
    for (int i = 0; i < TL[q].size(); i++) if (V[TL[q][i]] == 1) return true;
    return false;
}

void create_graph()
{
    vector< vector<int> > M = vector< vector<int> >(R, vector<int>(R, 0));
    G = vector< vector<int> > (R, vector<int>());
    for (int i = 0; i < R - 1; i++)
    {
        for (int j = i + 1; j < R; j++)
        {
            for (int k = 0; k < PpR; k++)
            {
                for (int l = 0; l < PpR; l++)
                {
                    if (intersect(i * PpR + k, j * PpR + l) == true)
                    {
                        M[i][j] = 1;
                        M[j][i] = 1;
                    }
                }
            }
        }
    }
    for (int i = 0; i < R; i++)
    {
        for (int j = 0; j < R; j++)
        {
            if (M[i][j] == 1) G[i].push_back(j);
        }
    }
}


*/
