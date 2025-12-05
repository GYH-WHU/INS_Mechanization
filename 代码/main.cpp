#include"IMU_Structs.h"

int main()
{
    int mode = 0;   // 解算模式 0示例数据 1自己数据
    cout << "Please choose the mode of solution:\n  0 ExampleData\n  1 SelfData\n";
    cin >> mode;
    if (!(mode == 0 || mode == 1)) { cout << "Error type of the solution!\n"; return 1; }

    switch (mode)
    {
    case 0:
    {
        cout << "\nNow is the solution of Example! Calculating...\n";

        // 打开文件
        FILE* m_imu = fopen("Data/Example_Data/IMU.bin", "rb");                 // 二进制形式打开示例文件，原始数据
        FILE* m_ins = fopen("Data/Example_Data/PureIns.bin", "rb");             // 二进制形式打开示例文件，参考数据

        FILE* result_example = fopen("Result/ExampleResult/INS_Result.txt", "wb");  // 写入解算结果
        FILE* true_example = fopen("Result/ExampleResult/INS_True.txt", "wb");      // 写入参考真值
        FILE* diff_result_example = fopen("Result/ExampleResult/INS_Diff_Result.txt", "wb");    // 写入差分结果
        FILE* denu_result = fopen("Result/ExampleResult/denu_Result.txt", "wb");    // 写入轨迹结果

        if (!(m_imu)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
        if (!(m_ins)) { cerr << "Cannot open the file to read rawdata!\n"; return -1; }
        if (!(result_example)) { cerr << "Cannot open the file to save result!\n"; return -1; }
        if (!(true_example)) { cerr << "Cannot open the file to save result!\n"; return -1; }
        if (!(diff_result_example)) { cerr << "Cannot open the file to save diff result!\n"; return -1; }
        if (!(denu_result)) { cerr << "Cannot open the file to save denu result!\n"; return -1; }
        fprintf(result_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");    // 写保存文件头
        fprintf(true_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");    // 写保存文件头
        fprintf(diff_result_example, "# TimeStamp,dVel(m/s),dBlh(deg),dPos_roll_pitch_yaw(deg)\n");    // 写保存文件头
        fprintf(denu_result, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");    // 写保存文件头

        // 变量
        IMUDataEpoch mImuDataEpoch_prv, mImuDataEpoch_pprv; // IMU原始数据（上个、上上个）
        INSDataEpoch result_prv;    // 计算结果（上个）
        INSDataEpoch result_pprv;   // 计算结果（上上个）
        Quater Q_prv;   // 四元数（上个）

        POSITION ZeroPoint; // 计算denu的零点
        double basexyz[3] = { 0.0,0.0,0.0 };
        ZeroPoint.latitude = initial_latitude; ZeroPoint.longitude = initial_longitude; ZeroPoint.H = initial_elevation;
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        bool flag_imu = true;       // 第一次启动循环
        bool Is_prv = false;        // 上个历元是否有数据
        bool Is_pprv = false;       // 上上个历元是否有数据

        double caltime = 0.0;   // 解算数据的时长
        int calnum = 0;         // 用于美化输出界面
        // 启动解算
        while (flag_imu == true)
        {
            // 变量
            IMUDataEpoch mImuDataEpoch_cur; // IMU原始数据（当前历元）
            INSDataEpoch mInsDataEpoch_cur; // INS参考数据
            INSDataEpoch result_cur;    // 计算结果（当前）
            Quater Q_cur;   // 四元数（当前）

            // 读取参考数据
            if (flag_imu) flag_imu = ReadExamplePureIMUData(m_imu, &mImuDataEpoch_cur);
            if (mImuDataEpoch_cur.TimeStamp < 91619.995)continue;   // 先舍弃前面的历元（后续可用于标定）

            if (!Is_prv)    // 判断上个历元的数据是否存下来了
            {
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_prv = true;  // 有数据了
                continue;
            }
            else if (!Is_pprv)   // 判断上上个历元的数据是否存下来了
            {
                mImuDataEpoch_pprv = mImuDataEpoch_prv;
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_pprv = true;  // 上上个有数据了

                // 开始解算的时候初始化常量
                // 1. 速度初始化（不用初始化，开始的时候是0）
                // 2. 位置初始化
                result_prv.blh.latitude = initial_latitude; result_prv.blh.longitude = initial_longitude; result_prv.blh.H = initial_elevation;
                result_pprv.blh.latitude = initial_latitude; result_pprv.blh.longitude = initial_longitude; result_pprv.blh.H = initial_elevation;
                // 3. 姿态初始化
                result_prv.pos.roll = initial_roll;  result_prv.pos.pitch = initial_pitch;  result_prv.pos.yaw = initial_heading;
                Q_prv.SetQbn(result_prv.pos);  // 设置上个四元数
                continue;
            }

            caltime += (mImuDataEpoch_cur.TimeStamp - mImuDataEpoch_prv.TimeStamp); // 统计解算时间
            if (caltime > 600.0)    // 每十分钟输出一次
            {
                caltime = 0.0;  // 清零
                calnum += 1;    // 加一

                printf("[%d] 已处理 10分钟 示例数据！\n", calnum);
            }

            /*** ！！！开始推算！！！ ***/

            // 读取参考数据（从91620.005开始的）
            ReadExamplePureINSData(m_ins, &mInsDataEpoch_cur);

            // 1. 先更新速度
            VelocityUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            // 2. 再更新位置
            PositionUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0,0.0,0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. 再更新姿态
            PostureUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0,0.0,0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;

            // 保存计算结果
            SaveOurResult(result_example, mImuDataEpoch_cur, result_cur.vel, result_cur.blh, result_cur.pos);

            if (mInsDataEpoch_cur.pos.yaw < 0)mInsDataEpoch_cur.pos.yaw += 360;
            SaveTrueResult(true_example, mInsDataEpoch_cur);

            // 保存差分结果
            double dvel[3] = { result_cur.vel.Vn - mInsDataEpoch_cur.vel.Vn, result_cur.vel.Ve - mInsDataEpoch_cur.vel.Ve, result_cur.vel.Vd - mInsDataEpoch_cur.vel.Vd };
            double dblh[3] = { blh[0] - mInsDataEpoch_cur.blh.latitude, blh[1] - mInsDataEpoch_cur.blh.longitude, blh[2] - mInsDataEpoch_cur.blh.H };
            if (POS[2] < 0)POS[2] += 360;
            double dpos[3] = { POS[0] - mInsDataEpoch_cur.pos.roll, POS[1] - mInsDataEpoch_cur.pos.pitch, POS[2] - mInsDataEpoch_cur.pos.yaw };
            if (fabs(mInsDataEpoch_cur.TimeStamp - mInsDataEpoch_cur.TimeStamp) < 1e-3)
                SaveDiffResult(diff_result_example, mImuDataEpoch_cur, dvel, dblh, dpos);
            else {
                cout << "The timestamp of truedata and ours isnt syn!\n";
            }
            // 保存轨迹结果
            double truexyz[3] = { 0.0,0.0,0.0 };
            double ourxyz[3] = { 0.0,0.0,0.0 };
            dENU truedenu, ourdenu;
            mInsDataEpoch_cur.blh.latitude *= Rad; mInsDataEpoch_cur.blh.longitude *= Rad;

            BLHToXYZ(mInsDataEpoch_cur.blh, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result, mImuDataEpoch_cur, truedenu, ourdenu);

            // 4. 保存当前历元数据
            mImuDataEpoch_pprv = mImuDataEpoch_prv; // 保存原始数据
            mImuDataEpoch_prv = mImuDataEpoch_cur;

            result_pprv = result_prv;   // 保存上个结果
            result_prv = result_cur;    // 保存当前结果
            Q_prv = Q_cur;      // 保存当前姿态四元数
        }

        cout << "示例数据解算完毕，解算结果、差分以及轨迹结果均保存在文件中！\n";
        // 关闭文件
        fclose(m_imu);
        fclose(m_ins);
        fclose(result_example);
        fclose(true_example);
        fclose(diff_result_example);
        fclose(denu_result);
        break;  // case的break
    }
    case 1:
    {
        cout << "Now is solution of SelfData! Calculating...\n";

        // 打开文件
        string ourdata_file_path = "Data\\GroupOne.ASC";    // 自采数据
        string truth_file_path = "Data\\TruthOne.nav";      // 参考真值

        ifstream ourdata_file(ourdata_file_path);
        ifstream truth_file(truth_file_path);
        if (!ourdata_file) { cerr << "Cannot open the file to read our data!\n"; return -1; }
        if (!truth_file) { cerr << "Cannot open the file to read truth data!\n"; return -1; }

        FILE* result_file = fopen("Result/result.txt", "wb");   // 写入计算结果
        if(!result_file) { cerr << "Cannot open the file to save results!\n"; return -1; }
        fprintf(result_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");    // 写保存文件头
        
        FILE* true_file = fopen("Result/true.txt", "wb");   // 重写参考真值
        if(!true_file) { cerr << "Cannot open the file to save true data!\n"; return -1; }
        fprintf(true_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");    // 写保存文件头

        FILE* diff_result_file = fopen("Result/diff_result.txt", "wb");   // 写入差分结果
        if(!diff_result_file) { cerr << "Cannot open the file to save diff results!\n"; return -1; }
        fprintf(diff_result_file, "# TimeStamp,dVel(m/s),dBLH(deg),dPos(deg)\n");    // 写保存文件头

        FILE* denu_result_file = fopen("Result/denu_result.txt", "wb");   // 写入轨迹结果
        if(!denu_result_file) { cerr << "Cannot open the file to save denu results!\n"; return -1; }
        fprintf(denu_result_file, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");    // 写保存文件头

        // 定义过去变量
        IMUDataEpoch ourdata_prv;   // 我们的原始数据（上个）
        IMUDataEpoch ourdata_pprv;  // 我们的原始数据（上上个）
        INSDataEpoch result_prv;    // 计算结果（上个）
        INSDataEpoch result_pprv;   // 计算结果（上上个）
        Quater Q_prv;   // 四元数（上个）

        INSDataEpoch truedata;    // 存放真值数据

        TimeIntervalsArray ZeroSpeed;   // 零速修正时间区间

        POSITION ZeroPoint; // 计算denu的零点
        double basexyz[3] = { 0.0,0.0,0.0 };
        //ZeroPoint.latitude = ours_initial_latitude; ZeroPoint.longitude = ours_initial_longitude; ZeroPoint.H = ours_initial_height;
        ZeroPoint.latitude = 30.527908355 * Rad; ZeroPoint.longitude = 114.355645940 * Rad; ZeroPoint.H = 23.3442;  // 用rtkplot计算的零点
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        bool Is_prv = false;    // 上一个历元是否存数据（false为没有）
        bool Is_pprv = false;   // 上上个历元是否存数据（false为没有）
        bool flag_imu = true;   // 第一次启动循环
        bool Is_Cali = false;   // 是否启用标定
        bool Is_Zero = false;   // 是否启用零速修正

        double epochnum = 0.0;  // 求平均时用的历元数
        double accmean[3] = { 0.0,0.0,0.0 }, gyrmean[3] = { 0.0,0.0,0.0 };  // 加速度计与角速度的平均值

        double caltime = 0.0;   // 解算时长
        int calnum = 0;         // 用于美化输出界面

        // 启动解算
        while (flag_imu == true)
        {
            // 定义当前变量
            IMUDataEpoch ourdata_cur;   // 我们的数据（当前）
            INSDataEpoch result_cur;    // 计算结果（当前）
            Quater Q_cur;   // 四元数（当前）

            // 读取实验数据
            DeviceType PureIMU_One = CGI;   // 纯惯导使用的设备类型
            DeviceType PureIMU_Five = XWGI; // 纯惯导使用的设备类型
            if (flag_imu)flag_imu = ReadIMURawData_CGI(ourdata_file, &ourdata_cur, PureIMU_One);    // 读取我们的数据
            if (ourdata_cur.TimeStamp > endtime)break;

            // 标定加速度计、陀螺的零偏(未使用）
            if (ourdata_cur.TimeStamp < starttime - 0.02)
            {
                // 计算平均值
                CalAvgAcc_Gyr(ourdata_cur, &epochnum, accmean, gyrmean);

                continue;
            }

            // 标定
            if (Is_Cali == true)
            {
                //printf("%0.8f %0.8f %0.8f %0.8f %0.8f %0.8f", ourdata_cur.Acc.X, ourdata_cur.Acc.Y, ourdata_cur.Acc.Z, ourdata_cur.Gyr.X, ourdata_cur.Gyr.Y, ourdata_cur.Gyr.Z);
                AccCalibration(accmean, &ourdata_cur);
                //GyrCalibration(gyrmean, &ourdata_cur);
                //printf(" || %0.8f %0.8f %0.8f %0.8f %0.8f %0.8f\n", ourdata_cur.Acc.X, ourdata_cur.Acc.Y, ourdata_cur.Acc.Z, ourdata_cur.Gyr.X, ourdata_cur.Gyr.Y, ourdata_cur.Gyr.Z);
            }

            // 检查前两个历元是否存下来了数据
            if (!Is_prv)    // 上一个历元没有存数据的时候
            {
                ourdata_prv = ourdata_cur;
                Is_prv = true;  // 已存数据
                continue;
            }
            else if(!Is_pprv)   // 上上个历元没有数据
            {
                ourdata_pprv = ourdata_prv;
                ourdata_prv = ourdata_cur;
                Is_pprv = true; // 已存数据

                // 之后就开始解算，此时初始化常量
                /* 1. 速度初始化 */
                /* 2. 位置初始化 */
                result_prv.blh.latitude = ours_initial_latitude; result_prv.blh.longitude = ours_initial_longitude; result_prv.blh.H = ours_initial_height;
                result_pprv.blh.latitude = ours_initial_latitude; result_pprv.blh.longitude = ours_initial_longitude; result_pprv.blh.H = ours_initial_height;
                /* 3. 姿态初始化 */
                result_prv.pos.roll = ours_initial_roll; result_prv.pos.pitch = ours_initial_pitch; result_prv.pos.yaw = ours_initial_yaw;
                /* 4. 设置上个历元的四元数 */
                Q_prv.SetQbn(result_prv.pos);

                continue;
            }

            // 美化输出
            caltime += (ourdata_cur.TimeStamp - ourdata_prv.TimeStamp); // 统计解算时长
            if (caltime >= 600.0)   //解算时长大于10分钟
            {
                caltime = 0.0;  // 清零
                calnum += 1;    // 加一

                printf("[%d] 已处理 10分钟 自采数据！\n", calnum);
            }

            /*** ！！！开始推算！！！ ***/
            ReadTruthData(truth_file, &truedata);   // 读取真值的数据
            while (truedata.TimeStamp < starttime)  // 同步时间
            {
                ReadTruthData(truth_file, &truedata);   // 读取真值的数据
            }

            result_cur.TimeStamp = ourdata_cur.TimeStamp;   // 复制时间

            // 1. 更新速度
            VelocityUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            if (Is_Zero == true)    // 是否进行零速修正
            {
                // 零速修正
                for (int i = 0; i < zero_time_intervals_num; i++)
                {
                    if (ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end)
                    {
                        ZeroSpeed.getInterval(i).used = true;   // 已经进行过零速修正的内容
                        continue;
                    }

                    if (i < zero_time_intervals_num - 1 &&
                        ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end &&
                        ourdata_cur.TimeStamp < ZeroSpeed.getInterval(i + 1).start) // 如果在某两区间的中间，则直接退出
                        break;

                    if (ourdata_cur.TimeStamp >= ZeroSpeed.getInterval(i).start &&
                        ourdata_cur.TimeStamp <= ZeroSpeed.getInterval(i).end)  // 在某区间内
                    {
                        result_cur.vel.Vn = 0.0;
                        result_cur.vel.Ve = 0.0;
                        result_cur.vel.Vd = 0.0;
                        break;
                    }
                }
            }

            // 2. 更新位置
            PositionUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0,0.0,0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. 更新姿态
            PostureUpdate(ourdata_cur, ourdata_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0,0.0,0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;
            if (POS[2] < 0)POS[2] += 360;

            // ① 保存计算结果
            if (fabs(ourdata_cur.TimeStamp - truedata.TimeStamp) < 1e-3 && ourdata_cur.TimeStamp <= endtime)
            {
                SaveOurResult(result_file, ourdata_cur, result_cur.vel, result_cur.blh, result_cur.pos);
                SaveTrueResult(true_file, truedata);
            }
            else {
                printf("解算时间：%0.3f 参考时间：%0.3f 没有同步！\n", ourdata_cur.TimeStamp, truedata.TimeStamp);
            }
            // ② 保存差分结果
            if (fabs(truedata.TimeStamp - result_cur.TimeStamp) < 1e-3 && ourdata_cur.TimeStamp <= endtime)
            {
                double dvel[3] = { result_cur.vel.Vn - truedata.vel.Vn,result_cur.vel.Ve - truedata.vel.Ve, result_cur.vel.Vd - truedata.vel.Vd };
                double dblh[3] = { blh[0] - truedata.blh.latitude,blh[1] - truedata.blh.longitude,blh[2] - truedata.blh.H };
                double dpos[3] = { POS[0] - truedata.pos.roll,POS[1] - truedata.pos.pitch,POS[2] - truedata.pos.yaw };

                if (dpos[2] > 180)dpos[2] -= 360;
                if (dpos[2] < -180)dpos[2] += 360;
                SaveDiffResult(diff_result_file, ourdata_cur, dvel, dblh, dpos);
            }
            else {
                cout << "The timestamp of truedata and ours isnt syn!\n";
            }
            // ③ 保存轨迹结果
            double truexyz[3] = { 0.0,0.0,0.0 };
            double ourxyz[3] = { 0.0,0.0,0.0 };
            dENU truedenu, ourdenu;

            truedata.blh.latitude *= Rad; truedata.blh.longitude *= Rad;
            BLHToXYZ(truedata.blh, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result_file, ourdata_cur, truedenu, ourdenu);

            // 4. 保存当前数据
            ourdata_pprv = ourdata_prv; // 保存原始数据
            ourdata_prv = ourdata_cur;

            result_pprv = result_prv;   // 保存上个结果
            result_prv = result_cur;    // 保存当前结果
            Q_prv = Q_cur;      // 保存当前姿态四元数
        }

        // 输出零速修正区间
        for (int i = 0; i < zero_time_intervals_num; i++)
        {
            if (ZeroSpeed.getInterval(i).used == true)
                printf("零速区间：%d %0.f - %0.f 用时：%0.f\n", i + 1,
                    ZeroSpeed.getInterval(i).start, ZeroSpeed.getInterval(i).end,
                    ZeroSpeed.getInterval(i).end - ZeroSpeed.getInterval(i).start);
        }

        cout << "自采数据解算完毕，解算结果、差分结果以及轨迹结果均保存在文件中！\n";
        // 关闭文件
        ourdata_file.close();
        truth_file.close();
        fclose(result_file);
        fclose(true_file);
        fclose(diff_result_file);
        fclose(denu_result_file);

        break;  // case的break
    }
    }

    return 0;
}