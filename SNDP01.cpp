// SNDP01.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include"D:\A_smartWorderrack\MPORJSP\Program C++\self_Define_headFile\Use_Curobi_Win64_C++.h"
#include"D:\A THESIS-SNDZROC\C++ Programming\SNDP01\sndphead.h"
using namespace std;
int main()
{
    int a, v, g, z, k, l, t;
    //获取需求
    vector<D> K;
    vector<D> G;
    string filename;
    filename = "demand.csv";
    getdemand(filename, &K, &G);
    //获取资源（车队信息）
    vector<vector<V>> fleets;
    filename = "fleettype.csv";
    fleets = getfleets(filename);

    //获取站点列表
    vector<int> terminals;
    filename = "terminals.csv";
    getterminals(filename, &terminals);


    // 获取物理网络
    vector<NODE> Pnet;
    vector<NODE>::iterator pi;
    string disfile = "distance.csv";
    string prifile = "unitcost.csv";
    Pnet = getPnet(disfile, prifile);

    //生成时空网络
    int T = 14;    //设置计划周期
    TSNET tsnet;
    for (pi = Pnet.begin(); pi != Pnet.end(); pi++) {
        vector<TODE> Termc;
        for (t = 0; t < T; t++) {
            TODE slt;
            slt.lt[0] = pi->l;
            slt.lt[1] = t;
            slt.zone = pi->zone;
            cout << pi->l << ", " << t << ": ";
            vector<ARC>::iterator ai;
            for (ai = pi->out.begin(); ai != pi->out.end(); ai++) {
                vector<int> nlt;
                nlt.push_back(ai->l);
                nlt.push_back((t + ai->time) % T);
                cout << "(" << ai->l << ", " << (t + ai->time) % T << ")  ";
                //此处可以大于一个周期但不要超过两个周期，未做约束与检测

                slt.out.push_back(nlt);
            }
            Termc.push_back(slt);
            cout << endl;
        }
        tsnet.Nodes.push_back(Termc);
    }// 节点添加完毕


    //添加弧
    for (l = 0; l < terminals.size(); l++) {
        for (t = 0; t < T; t++) {
            TARC tarc;
            vector<vector<int>> outarc;
            outarc = tsnet.Nodes[l][t].out;
            if (Pnet[l].zone.size() <= 1) {
                tarc.ori[0] = l;
                tarc.ori[1] = t;
                tarc.des[0] = l;
                tarc.des[1] = (t + 1) % T;
                tarc.price = 0.2;//每kg中转仓储费
                tarc.dist = 0;
                tarc.soh = 2;
                tarc.zone = intersection(Pnet[l].zone, Pnet[l].zone);
                tsnet.Arcs.push_back(tarc);

            }

            for (int oa = 0; oa < outarc.size(); oa++) {
                tarc.ori[0] = l;
                tarc.ori[1] = t;
                tarc.des[0] = outarc[oa][0];
                tarc.des[1] = outarc[oa][1];
                tarc.price = Pnet[l].out[oa].price;//每单元运输费
                tarc.dist = Pnet[l].out[oa].dist;
                tarc.soh = 1;
                tarc.zone = intersection(Pnet[l].zone, Pnet[outarc[oa][0]].zone);
                tsnet.Arcs.push_back(tarc);
            }
        }
    }

    tsnet.duplicate();

    //模型构建
    vector<int> Z;
    vector<int>::iterator zi;
    int A = tsnet.Arcs.size();
    GRBEnv* env = 0;
    GRBVar* m_g = 0;
    GRBVar** y_av = 0;
    GRBVar** eta_zv = 0;
    GRBVar** x_ka = 0;
    GRBVar** x_ga = 0;
    GRBConstr* c = 0;

    // Model
    env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_StringAttr_ModelName, "MZSNDP");

    // decision variables
    y_av = new GRBVar * [A];
    for (a = 0; a < A; a++) {
        vector<V>::iterator vi;
        Z = tsnet.Arcs[a].zone;
        for (zi = Z.begin(); zi != Z.end(); zi++) {
            v = fleets[*zi].size();
            y_av[a] = model.addVars(v, GRB_BINARY);
        }
    }

    m_g = model.addVars(G.size(), GRB_BINARY);


    eta_zv = new GRBVar * [fleets.size()];
    for (z = 0; z < fleets.size(); z++) {
        v = fleets[z].size();
        eta_zv[z] = model.addVars(v, GRB_BINARY);
    }

    x_ka = new GRBVar * [K.size()];
    for (k = 0; k < K.size(); k++) {
        x_ka[k] = model.addVars(A, GRB_CONTINUOUS);
    }


    x_ga = new GRBVar * [G.size()];
    for (g = 0; g < G.size(); g++) {
        x_ga[g] = model.addVars(A, GRB_CONTINUOUS);
    }


    // Set objective 
    GRBLinExpr C4, R2, C1, C2, C3, objExp;
    //车辆购置固定成本
    for (z = 0; z < fleets.size(); z++) {
        for (v = 0; v < fleets[z].size(); v++) {
            C1 += fleets[z][v].F * eta_zv[z][v];
        }
    }
    // 车辆相关的运输成本：
    for (a = 0; a < A; a++) {
        if (tsnet.Arcs[a].soh == 1) {
            vector<V>::iterator vi;
            Z = tsnet.Arcs[a].zone;
            for (zi = Z.begin(); zi != Z.end(); zi++) {
                for (v = 0; v < fleets[*zi].size(); v++) {
                    C2 += y_av[a][v] * tsnet.Arcs[a].dist * fleets[*zi][v].p;
                }
            }
        }
    }
    // 运量相关的运输与存储成本
    for (a = 0; a < A; a++) {
        for (k = 0; k < K.size(); k++) {
            C3 += tsnet.Arcs[a].price * x_ka[k][a];
        }
    }
    for (a = 0; a < A; a++) {
        for (g = 0; g < G.size(); g++) {
            C4 += tsnet.Arcs[a].price * x_ga[g][a];
        }
    }

    // 短期任务收入
    for (g = 0; g < G.size(); g++) {
        R2 += G[g].p * m_g[g];
    }

    objExp = C1 + C2 + C3 + C4 - R2; // 目标24
    model.setObjective(objExp, GRB_MINIMIZE);
    model.set(GRB_DoubleParam_TimeLimit, 300);

    // 约束25
    for (k = 0; k < K.size(); k++) {
        for (l = 0; l < terminals.size(); l++) {
            for (t = 0; t < T; t++) {
                ostringstream cname;
                cname << "25 k:" << k << " lt:" << l << "_" << t << "";
                GRBLinExpr le1, le2;
                for (a = 0; a < tsnet.Arcs.size(); a++) {
                    TARC ta = tsnet.Arcs[a];
                    bool b1 = (ta.ori[0] == l);
                    bool b2 = (ta.ori[1] == t);
                    bool b3 = (ta.des[0] == l);
                    bool b4 = (ta.des[1] == t);
                    if (b1 and b2) {
                        le1 += x_ka[k][a];
                    }
                    if (b3 and b4) {
                        le2 += x_ka[k][a];
                    }
                }
                bool b5 = (l == K[k].ori[0]);
                bool b6 = (t == K[k].ori[1]);

                bool b7 = (l == K[k].des[0]);
                bool b8 = (t == K[k].des[1]);

                if (b7 and b8) {// 如果lt是终点plus
                    cname << "plus";
                    model.addConstr(le1 - le2 == -K[k].w, cname.str());
                }
                else if (b5 and b6) {//如果lt是起点minus
                    cname << "minus";
                    model.addConstr(le1 - le2 == K[k].w, cname.str());
                }
                else {
                    cname << "else";
                    model.addConstr(le1 - le2 == 0, cname.str());
                }
            }
        }
    }

    // 约束26
    for (g = 0; g < G.size(); g++) {
        for (l = 0; l < terminals.size(); l++) {
            for (t = 0; t < T; t++) {
                ostringstream cname;
                cname << "26 g:" << g << " lt:" << l << "_" << t << "";
                GRBLinExpr le1, le2;
                for (a = 0; a < tsnet.Arcs.size(); a++) {
                    TARC ta = tsnet.Arcs[a];
                    bool b1 = (ta.ori[0] == l);
                    bool b2 = (ta.ori[1] == t);
                    bool b3 = (ta.des[0] == l);
                    bool b4 = (ta.des[1] == t);
                    if (b1 and b2) {
                        le1 += x_ga[g][a];
                    }
                    if (b3 and b4) {
                        le2 += x_ga[g][a];
                    }
                }
                bool b5 = (l == G[g].ori[0]);
                bool b6 = (t == G[g].ori[1]);
                bool b7 = (l == G[g].des[0]);
                bool b8 = (t == G[g].des[1]);

                if (b7 and b8) {        //如果lt是终点plus
                    cname << "plus";
                    model.addConstr(le1 - le2 == -G[g].w * m_g[g], cname.str());
                }
                else if (b5 and b6) {   //如果lt是起点minus
                    cname << "minus";
                    model.addConstr(le1 - le2 == G[g].w * m_g[g], cname.str());
                }
                else {
                    cname << "else";
                    model.addConstr(le1 - le2 == 0, cname.str());
                }
            }
        }
    }


    // 约束 28
    for (a = 0; a < A; a++) {
        if (tsnet.Arcs[a].soh == 1) {
            Z = tsnet.Arcs[a].zone;
            ostringstream cname;
            for (z = 0; z < Z.size(); z++) {
                GRBLinExpr le4;
                for (k = 0; k < K.size(); k++) {
                    le4 += x_ka[k][a];
                }
                GRBLinExpr le5;
                for (g = 0; g < G.size(); g++) {
                    le5 += x_ga[g][a];
                }

                GRBLinExpr le6;
                for (v = 0; v < fleets[Z[z]].size(); v++) {
                    le6 += y_av[a][v] * fleets[Z[z]][v].u;
                }
                cname << "27 a:" << a << "z" << Z[z];
                model.addConstr(le6 - le5 - le4 >= 0, cname.str());
            }

        }
    }

    // Add linear constraints 29
    for (z = 0; z < fleets.size(); z++) {
        for (v = 0; v < fleets[z].size(); v++) {
            for (t = 0; t < T - 1; t++) {
                GRBLinExpr le7;
                ostringstream cname;
                cname << "28 z" << z << "v" << v << "t" << t;
                for (a = 0; a < A; a++) {
                    Z = tsnet.Arcs[a].zone;
                    TARC ta = tsnet.Arcs[a];
                    for (int az = 0; az < Z.size(); az++) {
                        if (Z[az] == z) {
                            if ((ta.ori[1] <= t) and (ta.des[1] > t))le7 += y_av[a][v];
                        }
                    }
                }
                model.addConstr(le7 - eta_zv[z][v] == 0, cname.str());
            }
        }
    }

    // Add linear constraints 30
    for (l = 0; l < terminals.size(); l++) {
        for (t = 0; t < T; t++) {//lt


            for (z = 0; z < fleets.size(); z++) {
                for (v = 0; v < fleets[z].size(); v++) {

                    GRBLinExpr le8, le9;
                    ostringstream dname;
                    dname << "29 z" << z << "v" << v << "lt: " << l << "," << t;
                    for (a = 0; a < tsnet.Arcs.size(); a++) {
                        TARC ta = tsnet.Arcs[a];
                        Z = tsnet.Arcs[a].zone;
                        for (int az = 0; az < Z.size(); az++) {
                            if (Z[az] == z) {
                                bool b1 = (ta.ori[0] == l);
                                bool b2 = (ta.ori[1] == t);
                                
                                bool b3 = (ta.des[0] == l);
                                bool b4 = (ta.des[1] == t);
                                if (b1 and b2)le8 += y_av[a][v];
                                if (b3 and b4)le9 += y_av[a][v];
                            }
                        }
                    }

                    model.addConstr(le8 == le9, dname.str());
                }
            }
        }
    }

    model.optimize();

    int optimstatus = model.get(GRB_IntAttr_Status);
    cout << "Optimization complete" << endl;
    double objval = 0;
    if (optimstatus == GRB_OPTIMAL) {
        objval = model.get(GRB_DoubleAttr_ObjVal);
    }
    else if (optimstatus == GRB_INF_OR_UNBD) {
        cout << "Model is infeasible or unbounded" << endl;
        // do IIS
        cout << "The model is infeasible; computing IIS" << endl;
        model.computeIIS();
        cout << "\nThe following constraint(s) "
            << "cannot be satisfied:" << endl;
        c = model.getConstrs();
        for (int i = 0; i < model.get(GRB_IntAttr_NumConstrs); ++i)
        {
            if (c[i].get(GRB_IntAttr_IISConstr) == 1)
            {
                cout << c[i].get(GRB_StringAttr_ConstrName) << endl;
            }
        }
        return 0;

    }
    else if (optimstatus == GRB_INFEASIBLE) {
        cout << "Model is infeasible" << endl;
        // do IIS
        cout << "The model is infeasible; computing IIS" << endl;
        model.computeIIS();
        cout << "\nThe following constraint(s) "
            << "cannot be satisfied:" << endl;
        c = model.getConstrs();
        for (int i = 0; i < model.get(GRB_IntAttr_NumConstrs); ++i)
        {
            if (c[i].get(GRB_IntAttr_IISConstr) == 1)
            {
                cout << c[i].get(GRB_StringAttr_ConstrName) << endl;
            }
        }
        return 0;

    }
    else if (optimstatus == GRB_UNBOUNDED) {
        cout << "Model is unbounded" << endl;
        return 0;

    }
    else {
        cout << "Optimization was stopped with status = "
            << optimstatus << endl;
        return 0;

    }
   
    double xw, ol, ot, dl, dt, c1, c2;
    vector<vector <vector<TARC>>> z_service;
    for (z = 0; z < fleets.size(); z++) {
        vector <vector<TARC>> service;
        for (v = 0; v < fleets[z].size(); v++) {
            vector<TARC> route;
            if (eta_zv[z][v].get(GRB_DoubleAttr_X) >= 1) {
                xw = fleets[z][v].u;
                for (a = 0; a < tsnet.Arcs.size(); a++) {
                    Z = tsnet.Arcs[a].zone;
                    for (int az = 0; az < Z.size(); az++) {
                        if (Z[az] == z) {
                            if (y_av[a][v].get(GRB_DoubleAttr_X) >= 1) {   
                                TARC arc_v;
                                arc_v.ori[0] = tsnet.Arcs[a].ori[0]+1;
                                arc_v.ori[1] = tsnet.Arcs[a].ori[1]+1;
                                arc_v.des[0] = tsnet.Arcs[a].des[0]+1;
                                arc_v.des[1] = tsnet.Arcs[a].des[1]+1;
                                route.push_back(arc_v);
                            }
                        }
                    }
                } 
            }
            service.push_back(route);
        }
        z_service.push_back(service);
    }

    for (z = 0; z < fleets.size(); z++) {
        for (v = 0; v < fleets[z].size(); v++) {
            vector<TARC> route_v = z_service[z][v];
            if (route_v.size() > 0) {
                cout << "Z" << z << "V" << v << endl;
                int a = 0;
                int c = 0, b = 0, count_a = 0;
                cout << "(" << route_v[0].ori[0] << " " << route_v[0].ori[1] << ") ";
                cout << "(" << route_v[0].des[0] << " " << route_v[0].des[1] << ") ";
                count_a++;
                while (count_a < route_v.size()) {
                    for (b = 0; b < route_v.size();) {
                        if (route_v[a].des[0] == route_v[b].ori[0]) {
                            if (route_v[a].des[1] == route_v[b].ori[1]) {
                                cout << "(" << route_v[b].des[0] << " " << route_v[b].des[1] << ") ";
                                count_a++;
                                c = b;
                                break;
                            }
                        }
                        b++;
                    }
                  a = c;
                }
                cout << endl;
            
            }
            
        }
    }


    vector <vector<TARC>> k_service;
    for (k = 0; k < K.size(); k++) {
        vector<TARC> k_route;
        for (a = 0; a < tsnet.Arcs.size(); a++) {
            xw = x_ka[k][a].get(GRB_DoubleAttr_X);
            if (xw > 0.0001) {
                TARC arc_v;
                arc_v.ori[0] = tsnet.Arcs[a].ori[0] + 1;
                arc_v.ori[1] = tsnet.Arcs[a].ori[1] + 1;
                arc_v.des[0] = tsnet.Arcs[a].des[0] + 1;
                arc_v.des[1] = tsnet.Arcs[a].des[1] + 1;
                arc_v.price = xw;
                k_route.push_back(arc_v);
            }
        }
        k_service.push_back(k_route);
        cout << endl;
    }


    vector <vector<TARC>> g_service;
    for (g = 0; g < G.size(); g++) {
        vector<TARC> g_route;
        if (m_g[g].get(GRB_DoubleAttr_X) >= 1) {
            for (a = 0; a < tsnet.Arcs.size(); a++) {
                xw = x_ga[g][a].get(GRB_DoubleAttr_X);
                if (xw > 0.0001) {
                    TARC arc_v;
                    arc_v.ori[0] = tsnet.Arcs[a].ori[0] + 1;
                    arc_v.ori[1] = tsnet.Arcs[a].ori[1] + 1;
                    arc_v.des[0] = tsnet.Arcs[a].des[0] + 1;
                    arc_v.des[1] = tsnet.Arcs[a].des[1] + 1;
                    arc_v.price = xw;
                    g_route.push_back(arc_v);
                }
            }
        }
        g_service.push_back(g_route);
        cout << endl;
    }

    for (k = 0; k < K.size(); k++) {
        vector<TARC> route_v = k_service[k];
        if (route_v.size() > 0) {
            cout << "K" << k << endl;
            int a = 0;
            int c = 0, b = 0, count_a = 0;
            cout << "(" << K[k].ori[0]+1 << " " << K[k].ori[1]+1 << ") ";
            //
            int des_a = K[k].ori[0]+1;
            int dest_a= K[k].ori[1]+1; 
            count_a++;
            while (count_a < route_v.size()) {
                for (b = 0; b < route_v.size();) {
                    if (des_a == route_v[b].ori[0]) {
                        if (dest_a == route_v[b].ori[1]) {
                            cout << "(" << route_v[b].des[0] << " " << route_v[b].des[1] << ") " << route_v[b].price;
                            count_a++;
                            c = b;
                            break;
                        }
                    }
                    b++;
                }
                des_a = route_v[c].des[0];
                dest_a = route_v[c].des[1];

            }
            cout << "(" << K[k].des[0] + 1 << " " << K[k].des[1] + 1 << ") ";
            cout << endl;

        }
    
    }


    for (g = 0; g < G.size(); g++) {
        vector<TARC> route_v = g_service[g];
        if (route_v.size() > 0) {
            cout << "G" << g << endl;
            int a = 0;
            int c = 0, b = 0, count_a = 0;
            cout << "(" << G[g].ori[0]+1 << " " << G[g].ori[1]+1 << ") ";
            
            int des_a = G[g].ori[0]+1;
            int dest_a = G[g].ori[1]+1;
            count_a++;
            while (count_a < route_v.size()) {
                for (b = 0; b < route_v.size();) {
                    if (des_a == route_v[b].ori[0]) {
                        if (dest_a == route_v[b].ori[1]) {
                            cout << "(" << route_v[b].des[0] << " " << route_v[b].des[1] << ") " << route_v[b].price;
                            count_a++;
                            c = b;
                            break;
                        }
                    }
                    b++;
                }
                des_a = route_v[c].des[0];
                dest_a = route_v[c].des[1];
            }
            cout << "(" << G[g].des[0] + 1 << " " << G[g].des[1] + 1 << ") ";
            cout << endl;

        }

    }








    // 结果绘图
    
    char ins_name[] = "SNDR01";
    int Terminals = terminals.size();
    //double xw, ol, ot, dl, dt, c1, c2;
    int linew, lines;
    int counta = 0;
    char path[] = "gnuplot.exe"; //打开gnuplot
    char* gnuplotPath = path;
    FILE* gp = _popen(gnuplotPath, "w");
    if (gp == NULL)cout << ("Cannotopen gnuplot!\n") << endl;
       
    fprintf(gp, "set grid linewidth 4\n");
    //fprintf(gp, "set grid linewidth 4\n");
    fprintf(gp, "unset border\n");
    fprintf(gp, "set xlabel 'Terminals'\n");
    fprintf(gp, "set ylabel 'Periods' \n");
    fprintf(gp, "set xrange[-5:%d]\n", Terminals);
    fprintf(gp, "set yrange[0:%d]\n", T);
    fprintf(gp, "unset key\n");
    fprintf(gp, "set label '车辆购置固定总成本:%0.2f元' at %f,%f font',12'\n", C1.getValue(), -5.0, 4.5);
    fprintf(gp, "set label '车辆固定运输总成本:%0.2f元' at %f,%f front font',12'\n", C2.getValue(), -5.0, 3.5);
    fprintf(gp, "set label '长期需求变动运输总成本:%0.2f元' at %f,%f font',12'\n", C3.getValue(), -5.0, 2.5);
    fprintf(gp, "set label '短期需求变动运输总成本:%0.2f元' at %f,%f font',12'\n", C4.getValue(), -5.0, 1.5);
    fprintf(gp, "set label '短期需求总收入:%0.2f元' at %f,%f font',12'\n", R2.getValue(), -5.0, 0.5);
    cout << "车辆购置固定总成本:" << C1.getValue() << endl;
    cout << "车辆固定运输总成本:" << C2.getValue() << endl;
    cout << "长期需求变动运输总成本:" << C3.getValue() << endl;
    cout << "短期需求变动运输总成本:" << C4.getValue() << endl;
    cout << "短期需求总收入:" << R2.getValue() << endl;
 
    //y_av
    int countv = 0;
    for (z = 0; z < fleets.size(); z++) {
        cout << "区域：" << z<<",";
        for (v = 0; v < fleets[z].size(); v++) {
            if (eta_zv[z][v].get(GRB_DoubleAttr_X) >= 1) {
                xw = fleets[z][v].u;
                cout << "车："<<v << ",";
                for (a = 0; a < tsnet.Arcs.size(); a++) {
                    Z = tsnet.Arcs[a].zone;
                    for (int az = 0; az < Z.size(); az++) {
                        if (Z[az] == z) {
                            if (y_av[a][v].get(GRB_DoubleAttr_X) >= 1) {
                                
                                ol = tsnet.Arcs[a].ori[0];
                                ot = tsnet.Arcs[a].ori[1];
                                dl = tsnet.Arcs[a].des[0];
                                dt = tsnet.Arcs[a].des[1];
                                c1 = (ol + dl) * 0.5;
                                c2 = (ot + dt) * 0.5;
                                linew = 8;
                                lines = countv + 1;
                                cout << "a(" << ol+1<< " " << ot+1 << ":" << dl+1 << " " << dt+1 << "), ";
                                fprintf(gp, "set arrow from %f,%f to %f,%f lw %d lt %d \n", ol, ot, dl, dt, linew, lines);
                                // 运输量
                                if (counta == 0) {
                                    fprintf(gp, "set label 'Vz:%d_%d' at %f,%f right front font',12'\n", z, v, ol, ot);
                                    counta = 1;
                                }
                            }
                        }
                    }
                }
                counta = 0;
                countv++;
            }
            cout << endl;
        }
        cout << endl;
        fprintf(gp, "set title 'Vehicle Routes' font ',20'\n");
        fprintf(gp, "plot 0*x lt 0 notitle\n");
        fprintf(gp, "pause mouse\n");//用户点击后退出
    }

    fprintf(gp, "set title 'Vehicle Routes' font ',20'\n");
    fprintf(gp, "plot 0*x lt 0 notitle\n");
    fprintf(gp, "pause mouse\n");//用户点击后退出
    // x_ka
    for (k = 0; k < K.size(); k++) {
        cout << "常规需求：" << k << ",";
        for (a = 0; a < tsnet.Arcs.size(); a++) {
            xw = x_ka[k][a].get(GRB_DoubleAttr_X);
            if (xw > 0.0001) {
                
                ol = tsnet.Arcs[a].ori[0];
                ot = tsnet.Arcs[a].ori[1];
                dl = tsnet.Arcs[a].des[0];
                dt = tsnet.Arcs[a].des[1];
                srand(time(NULL));
                c1 = (ol + dl) * 0.6;
                c2 = (ot + dt) * 0.6;
                //弧
                linew = 4;
                lines = 8;
                cout << xw << "(" << ol+1 << " " << ot+1 << ":" << dl+1 << " " << dt+1 << "), ";
                fprintf(gp, "set arrow from %f, %f to %f, %f lw %d lt %d \n", ol, ot, dl, dt, linew, lines);
                // 运输量
                fprintf(gp, "set label 'K_%d:%0.1f' at %f, %f right front font',12'\n", k, xw, c1, c2);

            }
        }
        cout << endl;
        fprintf(gp, "set title 'Regular Shippments K%d' font ',20'\n", k);
        fprintf(gp, "plot 0*x lt 0 notitle\n");
        fprintf(gp, "pause mouse\n");//用户点击后退出
    }

    for (g = 0; g < G.size(); g++) {
        cout << "现货需求：" << g << ",";
        if (m_g[g].get(GRB_DoubleAttr_X) >= 1) {
            for (a = 0; a < tsnet.Arcs.size(); a++) {
                xw = x_ga[g][a].get(GRB_DoubleAttr_X);
                if (xw > 0.0001) {
                    ol = tsnet.Arcs[a].ori[0];
                    ot = tsnet.Arcs[a].ori[1];
                    dl = tsnet.Arcs[a].des[0];
                    dt = tsnet.Arcs[a].des[1];
                    srand(int(time(0)));
                    c1 = (ol + dl) * 0.5;
                    c2 = (ot + dt) * 0.5;
                    linew = 4;
                    lines = 7;
                    cout << xw << "(" << ol+1 << " " << ot+1 << ":" << dl+1 << " " << dt+1 << "), ";
                    fprintf(gp, "set arrow from %f, %f to %f, %f lw %d lt %d \n", ol, ot, dl, dt, linew, lines);
                    // 运输量
                    fprintf(gp, "set label 'G_%d:%0.1f' at %f, %f right front font',12'\n", k, xw, c1, c2);
                }
            }
        }
        cout << endl;
        fprintf(gp, "set title 'Temperal Shippments G%d' font ',20'\n", g);
        fprintf(gp, "plot 0*x lt 0 notitle\n");
        fprintf(gp, "pause mouse\n");//用户点击后退出

    }

    fprintf(gp, "pause mouse\n");//用户点击后退出
    _pclose(gp);
      





    return 0;
}