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
#define T 150
#define TpP 10
#define SD 5
#define N 100
#define TF false


vector< vector<int> > TL;
vector< vector<int> > G;
vector<int> OPT_S;
vector<int> OPT_A;

vector<int> OLD_S;
vector<int> OLD_A;

int rr = 0;

int A;

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

        xl = rx - 1000;
        xh = rx + 1000;
        yl = ry - 1000;
        yh = ry + 5000;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 0].push_back(j);
        }

        xl = rx - 1000;
        xh = rx + 1000;
        yl = ry - 5000;
        yh = ry + 1000;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 1].push_back(j);
        }

        xl = rx - 1000;
        xh = rx + 5000;
        yl = ry - 1000;
        yh = ry + 1000;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 2].push_back(j);
        }

        xl = rx - 5000;
        xh = rx + 1000;
        yl = ry - 1000;
        yh = ry + 1000;
        for (int j = 0; j < T; j++)
        {
            if (xl <= TP[j].x && TP[j].x <= xh && yl <= TP[j].y && TP[j].y <= yh)
                TL[i * PpR + 3].push_back(j);
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


//ordered greedy - robots are ordered by increasing max target coverage of single trajectory
int ordered_greedy_random()
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


    random_shuffle(RC.begin(), RC.end());
    random_shuffle(RC.begin(), RC.end());

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




int main()
{
    int num;

    ofstream myfile("data_1_geometric.txt");


    for (A = 3; A < 15; A += 3)
    {
        myfile << A << "\t";
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
        int rnd_average = 0;
        double rnd_max = -1.0;
        double rnd_min = 1000.0;
        vector<double> rnd_arr;



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

        while (num--)
        {
            construct_geometric();
 
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




            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int lsot = local_search_iog_atg();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            lsott += (et - st);
            lsot_average += lsot;
            lsot_arr.push_back(100.0*(double)lsot/old);
            if ((double)lsot/old > lsot_max) lsot_max = (double)lsot/old;
            if ((double)lsot/old < lsot_min) lsot_min = (double)lsot/old;
            cout << lsot << endl;



            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int lsnb = local_search_inv_abg();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            lsnbt += (et - st);
            lsnb_average += lsnb;
            lsnb_arr.push_back(100.0*(double)lsnb/old);
            if ((double)lsnb/old > lsnb_max) lsnb_max = (double)lsnb/old;
            if ((double)lsnb/old < lsnb_min) lsnb_min = (double)lsnb/old;
            cout << lsnb << endl;




            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int lsnt = local_search_inv_atg();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            lsntt += (et - st);
            lsnt_average += lsnt;
            lsnt_arr.push_back(100.0*(double)lsnt/old);
            if ((double)lsnt/old > lsnt_max) lsnt_max = (double)lsnt/old;
            if ((double)lsnt/old < lsnt_min) lsnt_min = (double)lsnt/old;
            cout << lsnt << endl;





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

            //cout << lnr << endl;



            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int cur = ordered_greedy_cur();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            curt += (et - st);
            cur_average += cur;
            cur_arr.push_back(100.0*(double)cur/old);
            if ((double)cur/old > cur_max) cur_max = (double)cur/old;
            if ((double)cur/old < cur_min) cur_min = (double)cur/old;
            //cout << lnr << endl;






            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int sc = ordered_greedy_sc();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            sct += (et - st);
            sc_average += sc;
            sc_arr.push_back(100.0*(double)sc/old);
            if ((double)sc/old > sc_max) sc_max = (double)sc/old;
            if ((double)sc/old < sc_min) sc_min = (double)sc/old;
            //cout << lnr << endl;



            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int scr = ordered_greedy_scr();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            scrt += (et - st);
            scr_average += scr;
            scr_arr.push_back(100.0*(double)scr/old);
            if ((double)scr/old > scr_max) scr_max = (double)scr/old;
            if ((double)scr/old < scr_min) scr_min = (double)scr/old;
            //cout << lnr << endl;






            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int mt = ordered_greedy_mt();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            mtt += (et - st);
            mt_average += mt;
            mt_arr.push_back(100.0*(double)mt/old);
            if ((double)mt/old > mt_max) mt_max = (double)mt/old;
            if ((double)mt/old < mt_min) mt_min = (double)mt/old;
            //cout << lnr << endl;



            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int mtr = ordered_greedy_mtr();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            mtrt += (et - st);
            mtr_average += mtr;
            mtr_arr.push_back(100.0*(double)mtr/old);
            if ((double)mtr/old > mtr_max) mtr_max = (double)mtr/old;
            if ((double)mtr/old < mtr_min) mtr_min = (double)mtr/old;
            //cout << lnr << endl;








            gettimeofday(&tps, NULL);
            st = tps.tv_sec * 1000000 + tps.tv_usec ;

            int rnd = ordered_greedy_random();

            gettimeofday(&tpe, NULL);
            et = tpe.tv_sec * 1000000 + tpe.tv_usec ;

            //mtrt += (et - st);
            rnd_average += rnd;


            cout << endl;
        }



        cout << "Old Algorithm" << endl;
        cout << "old:\t" << old_average / (double)N << endl;
        myfile << 100.0 * old_average / old_average << "\t";

        cout << "Oblivious Greedy Algorithm" << endl;
        cout << "obg:\t" << nv_average / (double)N << endl;
        myfile << 100.0 * nv_average / old_average << "\t";
        sort(nv_arr.begin(), nv_arr.end());
        myfile << nv_arr[74] << "\t";
        myfile << nv_arr[24] << "\t";

        cout << "Local Searchs" << endl;
        cout << "lsob:\t" << lsob_average / (double)N << endl;
        myfile << 100.0 * lsob_average / old_average << "\t";
        sort(lsob_arr.begin(), lsob_arr.end());
        myfile << lsob_arr[74] << "\t";
        myfile << lsob_arr[24] << "\t";
        cout << "lsot:\t" << lsot_average / (double)N << endl;
        myfile << 100.0 * lsot_average / old_average << "\t";
        sort(lsot_arr.begin(), lsot_arr.end());
        myfile << lsot_arr[74] << "\t";
        myfile << lsot_arr[24] << "\t";
        cout << "lsnb:\t" << lsnb_average / (double)N << endl;
        myfile << 100.0 * lsnb_average / old_average << "\t";
        sort(lsnb_arr.begin(), lsnb_arr.end());
        myfile << lsnb_arr[74] << "\t";
        myfile << lsnb_arr[24] << "\t";
        cout << "lsnt:\t" << lsnt_average / (double)N << endl;
        myfile << 100.0 * lsnt_average / old_average << "\t";
        sort(lsnt_arr.begin(), lsnt_arr.end());
        myfile << lsnt_arr[74] << "\t";
        myfile << lsnt_arr[24] << "\t";


        cout << "Ordered Greedy Algorithms" << endl;
        cout << "cu:\t" << cu_average / (double)N << endl;
        myfile << 100.0 * cu_average / old_average << "\t";
        sort(cu_arr.begin(), cu_arr.end());
        myfile << cu_arr[74] << "\t";
        myfile << cu_arr[24] << "\t";
        cout << "cur:\t" << cur_average / (double)N << endl;
        myfile << 100.0 * cur_average / old_average << "\t";
        sort(cur_arr.begin(), cur_arr.end());
        myfile << cur_arr[74] << "\t";
        myfile << cur_arr[24] << "\t";
        cout << "sc:\t" << sc_average / (double)N << endl;
        myfile << 100.0 * sc_average / old_average << "\t";
        sort(sc_arr.begin(), sc_arr.end());
        myfile << sc_arr[74] << "\t";
        myfile << sc_arr[24] << "\t";
        cout << "scr:\t" << scr_average / (double)N << endl;
        myfile << 100.0 * scr_average / old_average << "\t";
        sort(scr_arr.begin(), scr_arr.end());
        myfile << scr_arr[74] << "\t";
        myfile << scr_arr[24] << "\t";
        cout << "mt:\t" << mt_average / (double)N << endl;
        myfile << 100.0 * mt_average / old_average << "\t";
        sort(mt_arr.begin(), mt_arr.end());
        myfile << mt_arr[74] << "\t";
        myfile << mt_arr[24] << "\t";
        cout << "mtr:\t" << mtr_average / (double)N << endl;
        myfile << 100.0 * mtr_average / old_average << "\t";
        sort(mtr_arr.begin(), mtr_arr.end());
        myfile << mtr_arr[74] << "\t";
        myfile << mtr_arr[24] << "\t" << endl;


    }

    myfile.close();
    return 0;
}
