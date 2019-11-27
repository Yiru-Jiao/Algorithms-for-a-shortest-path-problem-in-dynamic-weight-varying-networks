function [T,T_best,P_best]=VWN(D,V,C,delta_t0,s,t)
% D distance距离矩阵（n*n），不相连的边为inf，n=size（1，D）为节点数
% V 速度矩阵（n*n），与D同结构，不相连的边为0
% C 交通灯周期（1*n）
% delta_t0 初始交通灯状态（1*n）
% s 起点节点
% t 终点节点
% T_best 最短时间
% P_best 最佳路径
n=size(D,1);%获取节点数
%-----------生成连接矩阵-----------
Connect=zeros(n);
for i=1:n
    num=1;
    for j=1:n
        if D(i,j)>0 && D(i,j)<inf
            Connect(i,num)=j;
        end
        num=num+1;
    end
end
%-------------------------------求得所有路径------------------------------------
paths=(1);
times=1;
while times<=n
    i=1;
    left=[];
    pos=[];
    while i<=size(paths,1)
        if paths(i,size(paths,2))==t||paths(i,size(paths,2))==inf
            pos0=inf;
        else
            pos0=find(Connect(paths(i,size(paths,2)),:)~=0)'; %与path中第i行末节点相接的节点标号
        end
        left=[left;repmat(paths(i,:),[size(pos0),1]);];
        pos=[pos;pos0];
        i=i+1;
    end
    paths=[left,pos];
    times=times+1;
end
m=size(paths,1);%所有路径条数
%----------------------------分别计算用时-----------------------------------
M=10000;
W=M*ones(n,n);% W 边权矩阵（n*n），初始化为M（任意大的正数）
delta_tj=zeros(n,n); % 辅助计算W，ij路段车辆出发时j交通等的状态
T=zeros(m,1);
for x=1:m
    %---------------------计算wij-----------------------
    i=paths(x,1);
    j=paths(x,2);
    while j~=inf
        if i==s  %计算delta_tj
            delta_tj(i,j)=delta_t0(j);
        else
            delta_tj(i,j)=C(j)-rem((T(x)+delta_t0(j)),C(j));
        end
        judge=rem((D(i,j)/V(i,j)+delta_tj(i,j)),C(j));%计算wij的判断条件
        if judge>=C(j)/2 && judge<C(j)  %计算并修改Wij
            W(i,j)=D(i,j)/V(i,j);
        else
            W(i,j)=D(i,j)/V(i,j)+C(j)/2-judge;
        end
        T(x)=T(x)+W(i,j);
        i=j;
        j=paths(x,find(paths(x,:)==i)+1);
    end
end
%--------------------------------------------------------------------------
Pos=find(T==min(T)); %找到最短时间的位置，当有重复时以最早出现的为准
P_best=paths(Pos(1),:) %最大迭代次数后最佳路径
T_best=T(Pos(1)) %最大迭代次数后最短时间
end

