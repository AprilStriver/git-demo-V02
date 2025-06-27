%% 谐波叠加
% % 路面参数量
% gq0 = 16e-6;    %A级路面
% gq0 = 64e-6;    %B级路面
gq0 = 256e-6;   %C级路面
% gq0 = 1024e-6;  %D级路面
% gq0 = 4096e-6;  %E级路面
% gq0 = 16384e-6; %F级路面
% gq0 = 65536e-6; %G级路面
% gq0 = 262144e-6;%H级路面
n0 = 0.1;       %参考空间频率 m^(-1)
w = 2;          %频率指数
n_min = 0.011;  %空间频率下限
n_max = 2.83;   %空间频率上限
dn = 0.001;     %空间频率分辨率
n = n_min:dn:n_max; %空间频率
counts = (n_max - n_min)/dn+1;%空间频率分段数
gqn = gq0*(n/n0).^(-w);%空间功率谱密度
L = 1000;%路面长度 m
dx = 0.02;
x  = 0:dx:L;%路面长度
counts2 = L/dx + 1;%路面分段数
A = zeros(1,counts);
wave = zeros(1,counts2);%临时存放某一空间频率处的路面不平度
qx = zeros(1,counts2);%路面不平度
load('theta.mat');
% theta = rand(1,counts) * 2 * pi;%随机相位0-2pi之间满足均匀分布的随机数
for k = 1:counts
    A(k) = sqrt(gqn(k)*dn);%某一空间频率处的路面幅值
    wave = sqrt(2) * A(k) * cos (2 * pi * n(k) * x + theta(k));
    qx = qx + wave;
end

%空间谱密度计算
nfft = 2048;
window = hanning(nfft);
noverlap = nfft/2;
[Pxx,n2] = pwelch(qx,window , noverlap , nfft, 1/dx);%1/dx为空间采样频率
rand_min = ceil(n_min * dx * nfft);
rand_max = ceil(n_max * dx * nfft);
%构造的谱密度图
% figure;
% loglog(n2(rand_min:rand_max),Pxx(rand_min:rand_max),'k');
% xlabel('空间频率/m^-^1','Fontsize',14,'FontWeight','bold')
% ylabel('功率/10^-^6m^3','Fontsize',14,'FontWeight','bold')
% hold on
% %理论谱密度图
% loglog(n,gqn,'red');
% grid;
% hold off
%时域路面不平度
v = 10;% 车速m/s
dt = dx./v;
time = x/v;
time = time';
xr = qx';
% figure
% plot(time , xr , 'k',time1(1:501),xr(1:501,1))
% xlabel('时间/s','Fontsize',14,'FontWeight','bold')
% ylabel('路面不平度/m','Fontsize',14,'FontWeight','bold')
% % legend('5m/s','10m/s')
% legend('20m/s','30m/s')
% axis([0 0.5 -0.02 0.06])
% figure
% plot(x,xr,'k');
% xlabel('路面/m','Fontsize',14,'FontWeight','bold')
% ylabel('路面不平度/m','Fontsize',14,'FontWeight','bold')
save E:\011_SimulinkWork\DOF2\xr xr;
save E:\011_SimulinkWork\DOF2\time time;
road = [x',1000*xr];
%% 定义1/4车辆二自由度模型参数 %%
% 系统参数
ms = 1135;    % 车体质量 kg
mt = 130;   % 车轮质量 kg
ks = 40500;  % 悬架刚度 N/m
kt = 850000; % 轮胎刚度 N/m
cs = 3450;   % 悬架被动阻尼 N·s/m
% co = 2500;  %悬架基础阻尼
% %% 定义1/4车辆二自由度模型控制参数 %%
% % % 以下是基准参数    ASH AADD sASH
% % 悬架天棚主动控制参数
% Famax_ASH = 10000;
% csky_ASH = 7000; 
% % 悬架天棚半主动控制参数
% cfmax_sASH =8000; 
% csky_sASH = 6000;
% co_sASH = 2500;  %基础阻尼
% % 悬架天棚ONOFF控制参数
% cfmax_ONOFF = 3000; 
% co_ONOFF = 2500;  %基础阻尼
% % 悬架加速度主动控制参数
% Famax_AADD = 10000;
% badd_AADD = 5000;
% % 悬架加速度半主动控制参数
% cfmax_sAADD = 6000;
% badd_sAADD = 1000;
% co_sAADD = 2500;
% % 悬架加速度ONOFF控制参数
% cfmax_SH_ADD = 1000;
% badd_SH_ADD = 500;
% co_SH_ADD = 2500;
% alpha = 4.2;
% % 悬架PID控制参数
% % PID控制器参数
% Kp = 500;   % PID 控制器比例系数
% Ki = 1000;  % PID 控制器积分系数
% Kd = 0.2;   % PID 控制器微分系数
% % PID半主动控制器参数
% cfmax_sAPID = 5000; % 磁流变阻尼器最大可调阻尼
% co_sAPID = 2500;
% % PID模糊控制器参数
% co_SaFPID = 2500;
%% 以下是调试参数    ASH  sASH  AADD
% 悬架天棚主动控制参数
Famax_ASH = 10000;
csky_ASH = 7000; 
co_ASH =3450;
% 悬架天棚半主动控制参数
cfmax_sASH =8000; 
csky_sASH = 6000;
co_sASH = 2500;  %基础阻尼
% 悬架天棚ONOFF控制参数
cfmax_ONOFF = 3000; 
co_ONOFF = 2500;  %基础阻尼
% 悬架加速度主动控制参数
Famax_AADD = 10000;
badd_AADD = 800;
co_AADD =3450;
% 悬架加速度半主动控制参数
cfmax_sAADD = 6000;
badd_sAADD = 1000;
co_sAADD = 2500;
% 悬架加速度ONOFF控制参数
cfmax_SH_ADD = 1000;
badd_SH_ADD = 500;
co_SH_ADD = 2500;
alpha = 4.2;
% 悬架PID控制参数
% PID控制器参数
Kp = 500;   % PID 控制器比例系数
Ki = 1000;  % PID 控制器积分系数
Kd = 0.2;   % PID 控制器微分系数
co_APID =2900;
% PID半主动控制器参数
cfmax_sAPID = 5000; % 磁流变阻尼器最大可调阻尼
co_sAPID = 2500;
% PID模糊控制器参数
co_SaFPID = 2500;
%% 以下是调试参数    ASH  sASH  APID
% % 悬架天棚主动控制参数
% Famax_ASH = 10000;
% csky_ASH = 7000; 
% % 悬架天棚半主动控制参数
% cfmax_sASH =8000; 
% csky_sASH = 6000;
% co_sASH = 2500;  %基础阻尼
% % 悬架天棚ONOFF控制参数
% cfmax_ONOFF = 3000; 
% co_ONOFF = 2500;  %基础阻尼
% % 悬架加速度主动控制参数
% Famax_AADD = 10000;
% badd_AADD = 1000;
% % 悬架加速度半主动控制参数
% cfmax_sAADD = 8000;
% badd_sAADD = 1000;
% co_sAADD = 2500;
% % 悬架加速度ONOFF控制参数
% cfmax_SH_ADD = 1000;
% badd_SH_ADD = 500;
% co_SH_ADD = 2500;
% alpha = 4.2;
% % 悬架PID控制参数
% % PID控制器参数
% Kp = 500;   % PID 控制器比例系数
% Ki = 1000;  % PID 控制器积分系数
% Kd = 0.2;   % PID 控制器微分系数
% % PID半主动控制器参数
% cfmax_sAPID = 5000; % 磁流变阻尼器最大可调阻尼
% co_sAPID = 2500;
% % PID模糊控制器参数
% co_SaFPID = 2500;
%% 路面输入 %%
load('time.mat'); % 等级路面时长
load('xr.mat');   % 等级路面不平度
CRoad = [time xr]; % 等级路面激励信号
% 加载模糊PID查表数据
load('fuzzy_pid_lookup.mat');  % 包含 Z_Kp, Z_Ki, Z_Kd, e_range, ec_range
sim DOF2_P_SH_ADD_PID_4_0611.slx;

%% 求均值 %%
rms_Passive_xxxs = sqrt(mean(xxxs(:,1).^2)); % 车体加速度的均方根
rms_Passive_xsxt = sqrt(mean(xsxt(:,1).^2)); % 悬架动行程的均方根
rms_Passive_xtxr = sqrt(mean(xtxr(:,1).^2)); % 车轮动行程的均方根

rms_ASH_xxxs = sqrt(mean(xxxs(:,2).^2)); % 车体加速度的均方根
rms_ASH_xsxt = sqrt(mean(xsxt(:,2).^2)); % 悬架动行程的均方根
rms_ASH_xtxr = sqrt(mean(xtxr(:,2).^2)); % 车轮动行程的均方根

rms_sASH_xxxs = sqrt(mean(xxxs(:,3).^2)); % 车体加速度的均方根
rms_sASH_xsxt = sqrt(mean(xsxt(:,3).^2)); % 悬架动行程的均方根
rms_sASH_xtxr = sqrt(mean(xtxr(:,3).^2)); % 车轮动行程的均方根

rms_sONOFF_xxxs = sqrt(mean(xxxs(:,4).^2)); % 车体加速度的均方根
rms_sONOFF_xsxt = sqrt(mean(xsxt(:,4).^2)); % 悬架动行程的均方根
rms_sONOFF_xtxr = sqrt(mean(xtxr(:,4).^2)); % 车轮动行程的均方根

rms_AADD_xxxs = sqrt(mean(xxxs(:,5).^2)); % 车体加速度的均方根
rms_AADD_xsxt = sqrt(mean(xsxt(:,5).^2)); % 悬架动行程的均方根
rms_AADD_xtxr = sqrt(mean(xtxr(:,5).^2)); % 车轮动行程的均方根

rms_sAADD_xxxs = sqrt(mean(xxxs(:,6).^2)); % 车体加速度的均方根
rms_sAADD_xsxt = sqrt(mean(xsxt(:,6).^2)); % 悬架动行程的均方根
rms_sADD_xtxr = sqrt(mean(xtxr(:,6).^2)); % 车轮动行程的均方根

rms_SHADD_xxxs = sqrt(mean(xxxs(:,7).^2)); % 车体加速度的均方根
rms_SHADD_xsxt = sqrt(mean(xsxt(:,7).^2)); % 悬架动行程的均方根
rms_SHADD_xtxr = sqrt(mean(xtxr(:,7).^2)); % 车轮动行程的均方根

rms_APID_xxxs = sqrt(mean(xxxs(:,8).^2)); % 车体加速度的均方根
rms_APID_xsxt = sqrt(mean(xsxt(:,8).^2)); % 悬架动行程的均方根
rms_APID_xtxr = sqrt(mean(xtxr(:,8).^2)); % 车轮动行程的均方根

rms_sAPID_xxxs = sqrt(mean(xxxs(:,9).^2)); % 车体加速度的均方根
rms_sAPID_xsxt = sqrt(mean(xsxt(:,9).^2)); % 悬架动行程的均方根
rms_sAPID_xtxr = sqrt(mean(xtxr(:,9).^2)); % 车轮动行程的均方根

rms_SaFAPID_xxxs = sqrt(mean(xxxs(:,10).^2)); % 车体加速度的均方根
rms_SaFAPID_xsxt = sqrt(mean(xsxt(:,10).^2)); % 悬架动行程的均方根
rms_SaFAPID_xtxr = sqrt(mean(xtxr(:,10).^2)); % 车轮动行程的均方根

% 策略名称
names = {
    'Passive';
    'ASH';
    'sASH';
    'sONOFF';
    'AADD';
    'sADD';
    'SHADD';
    'APID';
    'sAPID';
    'SaFAPID'
};

% RMS 原始值
acc_rms = [
    rms_Passive_xxxs;
    rms_ASH_xxxs;
    rms_sASH_xxxs;
    rms_sONOFF_xxxs;
    rms_AADD_xxxs;
    rms_sAADD_xxxs;
    rms_SHADD_xxxs;
    rms_APID_xxxs;
    rms_sAPID_xxxs;
    rms_SaFAPID_xxxs
];

sus_rms = [
    rms_Passive_xsxt;
    rms_ASH_xsxt;
    rms_sASH_xsxt;
    rms_sONOFF_xsxt;
    rms_AADD_xsxt;
    rms_sAADD_xsxt;
    rms_SHADD_xsxt;
    rms_APID_xsxt;
    rms_sAPID_xsxt;
    rms_SaFAPID_xsxt
];

whl_rms = [
    rms_Passive_xtxr;
    rms_ASH_xtxr;
    rms_sASH_xtxr;
    rms_sONOFF_xtxr;
    rms_AADD_xtxr;
    rms_sADD_xtxr;
    rms_SHADD_xtxr;
    rms_APID_xtxr;
    rms_sAPID_xtxr;
    rms_SaFAPID_xtxr
];


% 基准值：Passive（第1项）为参考
ref_acc = acc_rms(1);
ref_sus = sus_rms(1);
ref_whl = whl_rms(1);

% 计算优化比率（相对变化百分比）
acc_opt = (acc_rms - ref_acc) / ref_acc * 100;
sus_opt = (sus_rms - ref_sus) / ref_sus * 100;
whl_opt = (whl_rms - ref_whl) / ref_whl * 100;

% 合并成表格（保留 2位小数 & 带 % 符号）
opt_fmt = @(x) arrayfun(@(v) sprintf('%.1f%%', v), x, 'UniformOutput', false);

result_table = table( ...
    names, ...
    acc_rms, opt_fmt(acc_opt), ...
    sus_rms, opt_fmt(sus_opt), ...
    whl_rms, opt_fmt(whl_opt), ...
    'VariableNames', { ...
        '控制策略', ...
        '车体加速度', '加速度优化比率', ...
        '悬架动行程', '悬架优化比率', ...
        '车轮动行程', '车轮优化比率' ...
    });

% 显示到命令行
disp(result_table);

% 导出到 Excel
writetable(result_table, '控制策略对比_含优化比率.xlsx', 'WriteRowNames', false);



%% 求幅频特性 %%
% % 频率响应范围
% n1 = 0.01:0.005:0.21;
% n2 = 0.26:0.01:2.46;
% n = [n1,n2];
% % n = 0.01:0.02:2.41;%参考频率范围(空间频率)
% v = 10;%车速m/s
% f = n*v;%参考频率范围(时间频率)
% ln = length(f);%数据长度
% fcrossmax = max(f); % 设定为参考频率范围的最大值，或根据需求调整
% 
% delta = zeros(ln,1); % 确保 delta 预分配
% delta_xs = 100; % 初始化 delta_xs
% fcross = NaN; % 初始化交叉频率 fcross
% for j=1:ln
%     freq = f(j);
%     xr_sin = 0.05 * sin(2*pi*freq*time);
%     CRoad = [time xr_sin];
%     sim('DOF2_P_SH_ADD_PID_4_0429.slx')
%     rms_xr(j) = sqrt(mean(xr_sin.^2)); % 路面输入的均方根
%     rms_Passive_xxxs(j) = sqrt(mean(xxxs(:,1).^2)); % 车体加速度的均方根
%     rms_Passive_xsxt(j) = sqrt(mean(xsxt(:,1).^2)); % 悬架动行程的均方根
%     rms_Passive_xtxr(j) = sqrt(mean(xtxr(:,1).^2)); % 车轮动行程的均方根
% 
%     rms_ASH_xxxs(j) = sqrt(mean(xxxs(:,2).^2)); % 车体加速度的均方根
%     rms_ASH_xsxt(j) = sqrt(mean(xsxt(:,2).^2)); % 悬架动行程的均方根
%     rms_ASH_xtxr(j) = sqrt(mean(xtxr(:,2).^2)); % 车轮动行程的均方根
% 
%     rms_sASH_xxxs(j) = sqrt(mean(xxxs(:,3).^2)); % 车体加速度的均方根
%     rms_sASH_xsxt(j) = sqrt(mean(xsxt(:,3).^2)); % 悬架动行程的均方根
%     rms_sASH_xtxr(j) = sqrt(mean(xtxr(:,3).^2)); % 车轮动行程的均方根
% 
%     rms_sONOFF_xxxs(j) = sqrt(mean(xxxs(:,4).^2)); % 车体加速度的均方根
%     rms_sONOFF_xsxt(j) = sqrt(mean(xsxt(:,4).^2)); % 悬架动行程的均方根
%     rms_sONOFF_xtxr(j) = sqrt(mean(xtxr(:,4).^2)); % 车轮动行程的均方根
% 
%     rms_AADD_xxxs(j) = sqrt(mean(xxxs(:,5).^2)); % 车体加速度的均方根
%     rms_AADD_xsxt(j) = sqrt(mean(xsxt(:,5).^2)); % 悬架动行程的均方根
%     rms_AADD_xtxr(j) = sqrt(mean(xtxr(:,5).^2)); % 车轮动行程的均方根
% 
%     rms_sAADD_xxxs(j) = sqrt(mean(xxxs(:,6).^2)); % 车体加速度的均方根
%     rms_sAADD_xsxt(j) = sqrt(mean(xsxt(:,6).^2)); % 悬架动行程的均方根
%     rms_sAADD_xtxr(j) = sqrt(mean(xtxr(:,6).^2)); % 车轮动行程的均方根
% 
%     rms_SHADD_xxxs(j) = sqrt(mean(xxxs(:,7).^2)); % 车体加速度的均方根
%     rms_SHADD_xsxt(j) = sqrt(mean(xsxt(:,7).^2)); % 悬架动行程的均方根
%     rms_SHADD_xtxr(j) = sqrt(mean(xtxr(:,7).^2)); % 车轮动行程的均方根
% 
%     rms_APID_xxxs(j) = sqrt(mean(xxxs(:,8).^2)); % 车体加速度的均方根
%     rms_APID_xsxt(j) = sqrt(mean(xsxt(:,8).^2)); % 悬架动行程的均方根
%     rms_APID_xtxr(j) = sqrt(mean(xtxr(:,8).^2)); % 车轮动行程的均方根
% 
%     rms_sAPID_xxxs(j) = sqrt(mean(xxxs(:,9).^2)); % 车体加速度的均方根
%     rms_sAPID_xsxt(j) = sqrt(mean(xsxt(:,9).^2)); % 悬架动行程的均方根
%     rms_sAPID_xtxr(j) = sqrt(mean(xtxr(:,9).^2)); % 车轮动行程的均方根
% 
%     rms_SaFAPID_xxxs(j) = sqrt(mean(xxxs(:,10).^2)); % 车体加速度的均方根
%     rms_SaFAPID_xsxt(j) = sqrt(mean(xsxt(:,10).^2)); % 悬架动行程的均方根
%     rms_SaFAPID_xtxr(j) = sqrt(mean(xtxr(:,10).^2)); % 车轮动行程的均方根
% 
% 
%     H_Passive_xxxs(j) = rms_Passive_xxxs(j)/rms_xr(j);
%     H_Passive_xsxt(j) = rms_Passive_xsxt(j)/rms_xr(j);
%     H_Passive_xtxr(j) = rms_Passive_xtxr(j)/rms_xr(j);
% 
%     H_ASH_xxxs(j) = rms_ASH_xxxs(j)/rms_xr(j);
%     H_ASH_xsxt(j) = rms_ASH_xsxt(j)/rms_xr(j);
%     H_ASH_xtxr(j) = rms_ASH_xtxr(j)/rms_xr(j);
% 
%     H_sASH_xxxs(j) = rms_sASH_xxxs(j)/rms_xr(j);
%     H_sASH_xsxt(j) = rms_sASH_xsxt(j)/rms_xr(j);
%     H_sASH_xtxr(j) = rms_sASH_xtxr(j)/rms_xr(j);
% 
% 
%     H_sONOFF_xxxs(j) = rms_sONOFF_xxxs(j)/rms_xr(j);
%     H_sONOFF_xsxt(j) = rms_sONOFF_xsxt(j)/rms_xr(j);
%     H_sONOFF_xtxr(j) = rms_sONOFF_xtxr(j)/rms_xr(j);
% 
%     H_AADD_xxxs(j) = rms_AADD_xxxs(j)/rms_xr(j);
%     H_AADD_xsxt(j) = rms_AADD_xsxt(j)/rms_xr(j);
%     H_AADD_xtxr(j) = rms_AADD_xtxr(j)/rms_xr(j);
% 
%     H_sAADD_xxxs(j) = rms_sAADD_xxxs(j)/rms_xr(j);
%     H_sAADD_xsxt(j) = rms_sAADD_xsxt(j)/rms_xr(j);
%     H_sAADD_xtxr(j) = rms_sAADD_xtxr(j)/rms_xr(j);
% 
%     H_SHADD_xxxs(j) = rms_SHADD_xxxs(j)/rms_xr(j);
%     H_SHADD_xsxt(j) = rms_SHADD_xsxt(j)/rms_xr(j);
%     H_SHADD_xtxr(j) = rms_SHADD_xtxr(j)/rms_xr(j);
% 
%     H_APID_xxxs(j) = rms_APID_xxxs(j)/rms_xr(j);
%     H_APID_xsxt(j) = rms_APID_xsxt(j)/rms_xr(j);
%     H_APID_xtxr(j) = rms_APID_xtxr(j)/rms_xr(j);
% 
%     H_sAPID_xxxs(j) = rms_sAPID_xxxs(j)/rms_xr(j);
%     H_sAPID_xsxt(j) = rms_sAPID_xsxt(j)/rms_xr(j);
%     H_sAPID_xtxr(j) = rms_sAPID_xtxr(j)/rms_xr(j);
% 
%     H_SaFAPID_xxxs(j) = rms_SaFAPID_xxxs(j)/rms_xr(j);
%     H_SaFAPID_xsxt(j) = rms_SaFAPID_xsxt(j)/rms_xr(j);
%     H_SaFAPID_xtxr(j) = rms_SaFAPID_xtxr(j)/rms_xr(j);
% 
% 
% 
% end
% 
% %% 构建 H_all_9 数据结构（cell 嵌套 cell）
% H_all_9 = {
%     {H_Passive_xxxs, H_ASH_xxxs, H_sASH_xxxs, H_sONOFF_xxxs};
%     {H_Passive_xsxt, H_ASH_xsxt, H_sASH_xsxt, H_sONOFF_xsxt};
%     {H_Passive_xtxr, H_ASH_xtxr, H_sASH_xtxr, H_sONOFF_xtxr};
% 
%     {H_Passive_xxxs, H_AADD_xxxs, H_sAADD_xxxs, H_SHADD_xxxs};
%     {H_Passive_xsxt, H_AADD_xsxt, H_sAADD_xsxt, H_SHADD_xsxt};
%     {H_Passive_xtxr, H_AADD_xtxr, H_sAADD_xtxr, H_SHADD_xtxr};
% 
%     {H_Passive_xxxs, H_APID_xxxs, H_sAPID_xxxs, H_SaFAPID_xxxs};
%     {H_Passive_xsxt, H_APID_xsxt, H_sAPID_xsxt, H_SaFAPID_xsxt};
%     {H_Passive_xtxr, H_APID_xtxr, H_sAPID_xtxr, H_SaFAPID_xtxr};
% };
% 
% %% 设置绘图参数
% set(groot, 'defaultAxesFontName', 'Times New Roman');
% set(groot, 'defaultTextFontName', 'Times New Roman');
% set(groot, 'defaultLegendFontName', 'Times New Roman');
% set(groot, 'defaultAxesFontSize', 24);
% set(groot, 'defaultTextFontSize', 24);
% set(groot, 'defaultLegendFontSize', 22);
% 
% % 策略配色（对应 9 种控制策略）
% strategies = {'ASH','sASH','sONOFF','AADD','sAADD','SHADD','APID','sAPID','SaFAPID'};
% colors = [
%     1.0 0.5 0.0;   % ASH
%     1.0 0.8 0.0;   % sASH
%     0.5 0.0 0.5;   % sONOFF
%     0.2 0.6 1.0;   % AADD
%     0.4 0.8 0.6;   % sAADD
%     0.8 0.2 0.6;   % SHADD
%     0.3 0.3 0.8;   % APID
%     0.2 0.8 0.8;   % sAPID
%     0.9 0.5 0.1    % SaFAPID
% ];
% 
% % 控制组标签和变量索引
% control_groups = {
%     {'ASH','sASH','sONOFF'},
%     {'AADD','sAADD','SHADD'},
%     {'APID','sAPID','SaFAPID'}
% };
% group_names = {'ASH','ADD','PID'};
% 
% % 指标信息
% indicators = {'SMA', 'SWS', 'DTD'};
% ylabel_units = {'[s$^{-2}$]', '[\textendash]', '[\textendash]'};
% 
% %% 主循环绘图
% for i = 1:3  % 控制组
%     for j = 1:3  % 指标类型
% 
%         idx = (i - 1) * 3 + j; % 线性索引
%         H_cell = H_all_9{idx};
%         labels = [{'Passive'}, control_groups{i}{:}];
% 
%         % 获取颜色
%         color_set = [0 0 0];
%         for s = 1:3
%             ctrl_name = control_groups{i}{s};
%             clr_idx = find(strcmp(strategies, ctrl_name));
%             color_set = [color_set; colors(clr_idx,:)];
%         end
% 
%         % 标题和保存名
%         ind_str = indicators{j};
%         grp_str = group_names{i};
%         title_str = sprintf('Frequency Response: %s Strategies vs Passive', grp_str);
%         filebase = sprintf('%s_%s_FreqResp', ind_str, grp_str);
%         if strcmp(ind_str, 'SMA')
%     ylabel_str = ['$|H_{\mathrm{SMA}-X_r}|$ [s$^{-2}$]'];
% else
%     ylabel_str = ['$|H_{\mathrm{' ind_str '}-X_r}|$'];
% end
% 
%         % 绘图
%         figure('Name', title_str, 'NumberTitle', 'off', 'Position', [100,100,800,600]);
%         for k = 1:4
%             lw = 2; if k == 1, lw = 2.8; end
%             loglog(f, H_cell{k}, 'Color', color_set(k,:), 'LineWidth', lw); hold on;
%         end
% 
%         grid off;
%         xlabel('Frequency $f$ [Hz]', 'Interpreter','latex');
%         ylabel(ylabel_str, 'Interpreter','latex');
%         % title(title_str, 'FontSize', 22);
% 
%         lgd = legend(labels, 'Box','on');
%         set(lgd, 'Units','normalized', 'Position',[0.68 0.25 0.15 0.25], 'FontSize', 26);
%         set(gca, 'FontName','Times New Roman', 'FontSize', 28);
%         xlim([min(f) max(f)]);
%         set(gcf, 'PaperPositionMode','auto');
%                 % === 插入这里 ===
%         y_tick = get(gca, 'YTick');
%         y_lim = get(gca, 'YLim');
%         y_max_tick = max(y_tick);
%         y_max_lim = y_lim(2);
%         if abs(y_max_tick - y_max_lim) > 1e-6
%             y_tick = [y_tick y_max_lim];
%             set(gca, 'YTick', unique(y_tick));
%         end
% 
%         % 保存
%         saveas(gcf, [filebase, '.fig']);
%         print(gcf, [filebase, '.tif'], '-dtiff', '-r1200');
%         print(gcf, [filebase, '.eps'], '-depsc2', '-r1200');
%     end
% end
% %% ==== 方法2 提取10种控制策略的频域峰值（修改后的频段规则） ====
% 
% % 控制策略名称（含 Passive 共10项）
% controllers_all = {'Passive','ASH','sASH','sONOFF', ...
%                    'AADD','sAADD','SHADD', ...
%                    'APID','sAPID','SaFAPID'};
% 
% % 新指标列名（4个频段）
% indicator_names = {'G_peak_SMA_0_3','G_peak_SWS_0_3','G_peak_SWS_3up','G_peak_DTD_3up'};
% 
% % 定义频段索引（单位 Hz）
% idx_SMA     = find(f >= 0.0 & f <= 3.0);  % SMA: 0–3 Hz
% idx_SWS_0_3 = find(f >= 0.0 & f <= 3.0);  % SWS低频: 0–3 Hz
% idx_SWS_3up = find(f > 3.0);              % SWS高频: >3 Hz
% idx_DTD     = find(f > 3.0);              % DTD: >3 Hz
% 
% % 各指标频段索引组合（便于循环）
% band_idx = {idx_SMA, idx_SWS_0_3, idx_SWS_3up, idx_DTD};
% 
% % 初始化结果矩阵（10控制器 × 4频段）
% peak_values = zeros(10, 4);
% 
% for metric = 1:3  % 遍历 SMA、SWS、DTD 三个指标
%     row_idx = metric;
% 
%     % 提取三类控制器的指标响应（共10列）
%     H_SMA = H_all_9{row_idx};
%     H_ADD = H_all_9{row_idx + 3};
%     H_PID = H_all_9{row_idx + 6};
% 
%     H_combined = [H_SMA, H_ADD(2:4), H_PID(2:4)];
% 
%     if metric == 1  % SMA（0–3Hz）
%         idx_range = band_idx{1};
%         for i = 1:10
%             H_data = H_combined{i};
%             peak_values(i, 1) = max(H_data(idx_range));
%         end
% 
%     elseif metric == 2  % SWS（两个频段）
%         for subband = 1:2
%             idx_range = band_idx{subband + 1};  % SWS_0_3, SWS_3up
%             for i = 1:10
%                 H_data = H_combined{i};
%                 peak_values(i, subband + 1) = max(H_data(idx_range));
%             end
%         end
% 
%     elseif metric == 3  % DTD（>3Hz）
%         idx_range = band_idx{4};
%         for i = 1:10
%             H_data = H_combined{i};
%             peak_values(i, 4) = max(H_data(idx_range));
%         end
%     end
% end
% 
% % 构造表格输出
% peak_table_all = array2table(peak_values, ...
%     'VariableNames', indicator_names, ...
%     'RowNames', controllers_all);
% 
% disp('10种控制策略的频域峰值对比表（按新频段提取）：');
% disp(peak_table_all);


% ==== 计算 方法2 J^(k) 综合评分 ====

% % 整合成 [10×3] 的 RMS_all_3 矩阵（按 SMA, SWS, DTD）
% RMS_all_3 = [acc_rms, sus_rms, whl_rms];  % 与 peak_values 对应
% 
% % 从 peak_values 中提取各策略的峰值向量
% peak_SMA      = peak_values(:, 1);  % 10×1，SMA 在 0–3 Hz 区间的峰值
% peak_SWS_low  = peak_values(:, 2);  % 10×1，SWS 在 0–3 Hz 区间的峰值
% peak_SWS_high = peak_values(:, 3);  % 10×1，SWS 在 >3 Hz 区间的峰值
% peak_DTD      = peak_values(:, 4);  % 10×1，DTD 在 >3 Hz 区间的峰值
% 
% % --- 1) 提取被动控制器（Passive）作为归一化参考 ---
% RMS_ref       = RMS_all_3(1, :);       % 1×3
% G_ref_SMA     = peak_SMA(1);           % 标量
% G_ref_SWS_low = peak_SWS_low(1);       % 标量
% G_ref_SWS_high= peak_SWS_high(1);      % 标量
% G_ref_DTD     = peak_DTD(1);           % 标量
% 
% % --- 2) 定义权重（时域 & 频域） ---
% % 时域权重 w_T 对应 [SMA, SWS, DTD]
% w_T = [0.3, 0.15, 0.1];    % 行向量 1×3
% % 频域权重 w_F 对应 [SMA, SWS, DTD]
% % 其中 SWS 的总频域权重为 0.10，但其内部“双峰合并”时，先按 0.5/0.5 计算比值，再乘以 0.10
% w_F = [0.2, 0.10, 0.15];    % 行向量 1×3
% 
% % 维度检查：
% assert(isrow(w_T) && numel(w_T)==3, 'w_T 应为 1×3 行向量');
% assert(isrow(w_F) && numel(w_F)==3, 'w_F 应为 1×3 行向量');
% assert(size(RMS_all_3,2)==3, 'RMS_all_3 必须是 10×3 矩阵');
% assert(iscolumn(peak_SMA)       && numel(peak_SMA)==10,       'peak_SMA 应为 10×1 列向量');
% assert(iscolumn(peak_SWS_low)   && numel(peak_SWS_low)==10,   'peak_SWS_low 应为 10×1 列向量');
% assert(iscolumn(peak_SWS_high)  && numel(peak_SWS_high)==10,  'peak_SWS_high 应为 10×1 列向量');
% assert(iscolumn(peak_DTD)       && numel(peak_DTD)==10,       'peak_DTD 应为 10×1 列向量');
% 
% % --- 3) 初始化 J_score --- 
% numStrategies = size(RMS_all_3, 1);
% J_score = zeros(numStrategies, 1);
% 
% % --- 4) 逐策略计算归一化比值并求解综合评分 ---
% for i = 1:numStrategies
%     % 4.1) 提取当前策略的时域 RMS 值
%     RMS_i = RMS_all_3(i, :);  % 1×3 行向量
% 
%     % 4.2) 计算时域比值 => RMS_ratio
%     RMS_ratio = RMS_i ./ RMS_ref;  % 1×3 行向量
% 
%     % 4.3) 计算频域比值
%     % 4.3.1) SMA 频域比值
%     G_ratio_SMA = peak_SMA(i) / G_ref_SMA;  % 标量
% 
%     % 4.3.2) SWS 双峰比值：低频与高频各占 0.5 份，再整体乘以 w_F(2)
%     r1 = peak_SWS_low(i)  / G_ref_SWS_low;   % 低频比值
%     r2 = peak_SWS_high(i) / G_ref_SWS_high;  % 高频比值
%     G_ratio_SWS = 1 * r1 + 0 * r2;       % 合并后等价比值
% 
%     % 4.3.3) DTD 频域比值
%     G_ratio_DTD = peak_DTD(i) / G_ref_DTD;  % 标量
% 
%     % 4.3.4) 合并成 1×3 行向量
%     G_ratio = [G_ratio_SMA, G_ratio_SWS, G_ratio_DTD];
% 
%     % 4.4) 检查 G_ratio 维度
%     assert(isrow(G_ratio) && numel(G_ratio)==3, 'G_ratio 应为 1×3 行向量');
% 
%     % 4.5) 计算综合评分：J = sum( w_T .* RMS_ratio + w_F .* G_ratio )
%     J_score(i) = sum(w_T .* RMS_ratio + w_F .* G_ratio);
% end
% 
% % --- 5) 构造并排序评分表格，J 值越小表示性能越优 ---
% J_table = table(controllers_all(:), J_score, ...
%     'VariableNames', {'Control_Strategy', 'J_score'});
% J_table = sortrows(J_table, 'J_score');
% 
% % --- 6) 显示结果 ---
% disp('控制策略综合评分排序（J 值越小越优）：');
% disp(J_table);
%% ==== 最终版时域对比图绘制（颜色统一频域） ====

% === 原始数据 ===
data_all = {xxxs, xsxt, xtxr};  % 三个指标：SMA, SWS, DTD
ylabels  = {'SMA / m·s^{-2}', 'SWS / m', 'DTD / m'};
group_list = {
    'ASH',  {'Passive','ASH','sASH','sONOFF'},   [1 2 3 4];
    'AADD', {'Passive','AADD','sAADD','SHADD'},  [1 5 6 7];
    'PID',  {'Passive','APID','sAPID','SaFAPID'},[1 8 9 10];
};

% === 控制器颜色映射（频域统一策略） ===
strategies = {'ASH','sASH','sONOFF','AADD','sAADD','SHADD','APID','sAPID','SaFAPID'};
colors = [
    1.0 0.5 0.0;   % ASH
    1.0 0.8 0.0;   % sASH
    0.5 0.0 0.5;   % sONOFF
    0.2 0.6 1.0;   % AADD
    0.4 0.8 0.6;   % sAADD
    0.8 0.2 0.6;   % SHADD
    0.3 0.3 0.8;   % APID
    0.2 0.8 0.8;   % sAPID
    0.9 0.5 0.1    % SaFAPID
];
passive_color = [0 0 0];  % Passive 控制器颜色（黑）

% === 输出路径 ===
outpath = 'E:/011_SimulinkWork/DOF2/DOF2_TimeDomain_RMS/';
if ~exist(outpath, 'dir')
    mkdir(outpath);
end

% === 绘图主循环 ===
for g = 1:3  % 三组控制策略
    fig = figure('Position', [50, 0, 800, 950]);
    idx_set = group_list{g,3};
    legends = group_list{g,2};
    group_name = group_list{g,1};

    axs = gobjects(3,1);  % 记录3个子图句柄
    for i = 1:3
        axs(i) = subplot(3,1,i);
        y_data = data_all{i};
        Nt = min(size(y_data,1), length(time));
        t = time(1:Nt);
        [~, t5_idx] = min(abs(t - 5));
        t = t(1:t5_idx);  % 限定 0~5s
        hold on;
        for k = 1:4
            raw = y_data(1:t5_idx, idx_set(k));
            smooth_y = smoothdata(raw, 'loess', 10);  % 保持滤波
            lw = 0.8; if k==1, lw = 1; end  % 被动线加粗

            % === 获取颜色（统一频域）===
            if k == 1
                color_use = passive_color;
            else
                ctrl_name = legends{k};  % 如 'sAADD'
                clr_idx = find(strcmp(strategies, ctrl_name));
                color_use = colors(clr_idx, :);
            end

            plot(t, smooth_y, 'LineWidth', lw, 'Color', color_use);
        end

        ylabel(ylabels{i}, 'FontSize', 16);
        set(gca, 'FontName','Times New Roman', 'FontSize', 16);
        grid off; xlim([0 5]);
        if i==3
            xlabel('Time / s', 'FontSize', 16);
        end
    end

    % === 图例：底部横排 ===
    lgd = legend(axs(3), legends, 'Orientation','horizontal', ...
        'Location','southoutside', 'FontSize', 16, 'Box','on');

    % === 左对齐所有 Y 轴标签 ===
    positions = cell2mat(get(axs, 'Position'));
    min_left = min(positions(:,1));
    max_width = max(positions(:,3));
    for i = 1:3
        positions(i,1) = min_left;
        positions(i,3) = max_width;
        set(axs(i), 'Position', positions(i,:));
    end

    % === 保存文件 ===
    fname = sprintf('TDomain_%s_SMA_SWS_DTD', group_name);
    saveas(gcf, fullfile(outpath, [fname '.fig']));
    print(gcf, fullfile(outpath, [fname '.tif']), '-dtiff', '-r600');
    print(gcf, fullfile(outpath, [fname '.eps']), '-depsc2', '-r600');
end
