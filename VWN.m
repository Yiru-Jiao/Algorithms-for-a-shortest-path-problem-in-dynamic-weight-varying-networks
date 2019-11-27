function [T,T_best,P_best]=VWN(D,V,C,delta_t0,s,t)
% D distance�������n*n�����������ı�Ϊinf��n=size��1��D��Ϊ�ڵ���
% V �ٶȾ���n*n������Dͬ�ṹ���������ı�Ϊ0
% C ��ͨ�����ڣ�1*n��
% delta_t0 ��ʼ��ͨ��״̬��1*n��
% s ���ڵ�
% t �յ�ڵ�
% T_best ���ʱ��
% P_best ���·��
n=size(D,1);%��ȡ�ڵ���
%-----------�������Ӿ���-----------
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
%-------------------------------�������·��------------------------------------
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
            pos0=find(Connect(paths(i,size(paths,2)),:)~=0)'; %��path�е�i��ĩ�ڵ���ӵĽڵ���
        end
        left=[left;repmat(paths(i,:),[size(pos0),1]);];
        pos=[pos;pos0];
        i=i+1;
    end
    paths=[left,pos];
    times=times+1;
end
m=size(paths,1);%����·������
%----------------------------�ֱ������ʱ-----------------------------------
M=10000;
W=M*ones(n,n);% W ��Ȩ����n*n������ʼ��ΪM��������������
delta_tj=zeros(n,n); % ��������W��ij·�γ�������ʱj��ͨ�ȵ�״̬
T=zeros(m,1);
for x=1:m
    %---------------------����wij-----------------------
    i=paths(x,1);
    j=paths(x,2);
    while j~=inf
        if i==s  %����delta_tj
            delta_tj(i,j)=delta_t0(j);
        else
            delta_tj(i,j)=C(j)-rem((T(x)+delta_t0(j)),C(j));
        end
        judge=rem((D(i,j)/V(i,j)+delta_tj(i,j)),C(j));%����wij���ж�����
        if judge>=C(j)/2 && judge<C(j)  %���㲢�޸�Wij
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
Pos=find(T==min(T)); %�ҵ����ʱ���λ�ã������ظ�ʱ��������ֵ�Ϊ׼
P_best=paths(Pos(1),:) %���������������·��
T_best=T(Pos(1)) %���������������ʱ��
end

