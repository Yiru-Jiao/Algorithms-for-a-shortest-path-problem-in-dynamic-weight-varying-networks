function [ Tmin,path,T_ave,T_best,P_best ] = VWA( D,V,C,delta_t0,s,t,Nc_max,m,Q,alpha,beta,rho )
%=====================符号解释======================
% Tmin 每一代最短时间阵(Nc_max*1)
% path 每一代最短路径阵(Nc_max*n)
% T_best 最短时间
% P_best 最佳路径
% D distance距离矩阵（n*n），不相连的边为inf，n=size（1，D）为节点数
% V 速度矩阵（n*n），与D同结构，不相连的边为0
% C 交通灯周期（1*n）
% delta_t0 初始交通灯状态（1*n）
% s 起点节点
% t 终点节点
% Nc 迭代次数
% Nc_max 最大迭代次数
% m 蚂蚁个数
% Q 每只蚂蚁携带的信息素量
% alpha 0<alpha<1,信息素量更新系数
% beta 启发信息的作用系数
% rho 信息素挥发后剩余度
%===================初始化==========================
Nc=0;
n=size(D,1);%获取节点数
Tmin=inf*ones(Nc_max,1);
path=zeros(Nc_max,n);
T_ave=zeros(Nc_max,1);
Tau=ones(n,n);% Tau 信息素矩阵（n*n），初始均为1
M=10000;
W=M*ones(n,n);% W 边权矩阵（n*n），初始化为M（任意大的正数）
delta_tj=zeros(n,n); % 辅助计算W，ij路段车辆出发时j交通等的状态
%==================开始循环==========================
while Nc<Nc_max  %达到最大迭代次数时停止
    Nc=Nc+1;
    Tabu=zeros(m,n);   %存储并记录路径的生成
    Tabu(:,1)=s; %把蚂蚁放在起点处
    T=zeros(m,1); %存储并记录与Tabu对应的累计边权
    for k=1:m  % m只蚂蚁分别遍历
        i=s;
        num=1;
        while i~=t   %计算ij段的信息，到终点为止
            P=zeros(1,n);
            %================================对i节点=====================================
            for j=1:n %遍历每个节点
                    %--------------------------修改动态边权-----------------------------
                if D(i,j)>0 && D(i,j)<inf %判断与i相连的边vij
                    if i==s  %计算delta_tj
                        delta_tj(i,j)=delta_t0(j);
                    else
                        delta_tj(i,j)=C(j)-rem((T(k)+delta_t0(j)),C(j));
                    end 
                    judge=rem((D(i,j)/V(i,j)+delta_tj(i,j)),C(j));%计算wij的判断条件
                    if judge>=C(j)/2 && judge<C(j)  %计算并修改Wij
                        W(i,j)=D(i,j)/V(i,j);
                    else
                        W(i,j)=D(i,j)/V(i,j)+C(j)/2-judge;
                    end
                    %-------------------------------------------------------------------
                    Eta=1./W;
                    P(j)=Tau(i,j)^alpha*Eta(i,j)^beta;
                end
            end
            %=============================================================================
            P=P/sum(P); %计算pij
            Pcum=cumsum(P); %用轮盘赌来做随机选择
            Select=find(Pcum>=rand);
            i_next=Select(1); %确定i的转移
            Tau(i,i_next)=alpha*Tau(i,i_next);%更新被选中的边的信息素量
            Tabu(k,num+1)=i_next;%更新Tabu的m行
            T(k)=T(k)+W(i,i_next);%更新T的m行
            i=i_next;
            num=num+1;
        end
    end
    Tmin(Nc)=min(T);         %m只蚂蚁中的最短用时
    pos=find(T==Tmin(Nc));
    path(Nc,:)=Tabu(pos(1),:); %此轮迭代后的最佳路线，当有两条路径均为最短用时时，取先出发的那只蚂蚁的路径
    T_ave(Nc)=mean(T);
    %--------------------更新全局的信息素量----------------------------
    Delta_Tau=zeros(n,n);    %开始时信息素为n*n的0矩阵
    for i=1:m
        for j=1:(sum(Tabu(i,:)~=0,2)-1)  %size计算第i行路径的节点数
            Delta_Tau(Tabu(i,j),Tabu(i,j+1))=Delta_Tau(Tabu(i,j),Tabu(i,j+1))+Q/T(i);
            %在蚂蚁经过的路径（i，j）上的信息素增量
        end
    end
    Tau=rho.*Tau+Delta_Tau; %考虑信息素挥发，更新后的信息素
    %-----------------------------------------------------------------
    Tabu=zeros(m,n);
end
Pos=find(Tmin==min(Tmin)); %找到最短时间的位置，当有重复时以最早出现的为准
P_best=path(Pos(1),:) %最大迭代次数后最佳路径
T_best=Tmin(Pos(1)) %最大迭代次数后最短时间
plot(T_ave); %画每代平均时间的图像
title('历代平均时间')     
end

