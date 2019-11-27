function [ Tmin,path,T_ave,T_best,P_best ] = VWA( D,V,C,delta_t0,s,t,Nc_max,m,Q,alpha,beta,rho )
%=====================���Ž���======================
% Tmin ÿһ�����ʱ����(Nc_max*1)
% path ÿһ�����·����(Nc_max*n)
% T_best ���ʱ��
% P_best ���·��
% D distance�������n*n�����������ı�Ϊinf��n=size��1��D��Ϊ�ڵ���
% V �ٶȾ���n*n������Dͬ�ṹ���������ı�Ϊ0
% C ��ͨ�����ڣ�1*n��
% delta_t0 ��ʼ��ͨ��״̬��1*n��
% s ���ڵ�
% t �յ�ڵ�
% Nc ��������
% Nc_max ����������
% m ���ϸ���
% Q ÿֻ����Я������Ϣ����
% alpha 0<alpha<1,��Ϣ��������ϵ��
% beta ������Ϣ������ϵ��
% rho ��Ϣ�ػӷ���ʣ���
%===================��ʼ��==========================
Nc=0;
n=size(D,1);%��ȡ�ڵ���
Tmin=inf*ones(Nc_max,1);
path=zeros(Nc_max,n);
T_ave=zeros(Nc_max,1);
Tau=ones(n,n);% Tau ��Ϣ�ؾ���n*n������ʼ��Ϊ1
M=10000;
W=M*ones(n,n);% W ��Ȩ����n*n������ʼ��ΪM��������������
delta_tj=zeros(n,n); % ��������W��ij·�γ�������ʱj��ͨ�ȵ�״̬
%==================��ʼѭ��==========================
while Nc<Nc_max  %�ﵽ����������ʱֹͣ
    Nc=Nc+1;
    Tabu=zeros(m,n);   %�洢����¼·��������
    Tabu(:,1)=s; %�����Ϸ�����㴦
    T=zeros(m,1); %�洢����¼��Tabu��Ӧ���ۼƱ�Ȩ
    for k=1:m  % mֻ���Ϸֱ����
        i=s;
        num=1;
        while i~=t   %����ij�ε���Ϣ�����յ�Ϊֹ
            P=zeros(1,n);
            %================================��i�ڵ�=====================================
            for j=1:n %����ÿ���ڵ�
                    %--------------------------�޸Ķ�̬��Ȩ-----------------------------
                if D(i,j)>0 && D(i,j)<inf %�ж���i�����ı�vij
                    if i==s  %����delta_tj
                        delta_tj(i,j)=delta_t0(j);
                    else
                        delta_tj(i,j)=C(j)-rem((T(k)+delta_t0(j)),C(j));
                    end 
                    judge=rem((D(i,j)/V(i,j)+delta_tj(i,j)),C(j));%����wij���ж�����
                    if judge>=C(j)/2 && judge<C(j)  %���㲢�޸�Wij
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
            P=P/sum(P); %����pij
            Pcum=cumsum(P); %�����̶��������ѡ��
            Select=find(Pcum>=rand);
            i_next=Select(1); %ȷ��i��ת��
            Tau(i,i_next)=alpha*Tau(i,i_next);%���±�ѡ�еıߵ���Ϣ����
            Tabu(k,num+1)=i_next;%����Tabu��m��
            T(k)=T(k)+W(i,i_next);%����T��m��
            i=i_next;
            num=num+1;
        end
    end
    Tmin(Nc)=min(T);         %mֻ�����е������ʱ
    pos=find(T==Tmin(Nc));
    path(Nc,:)=Tabu(pos(1),:); %���ֵ���������·�ߣ���������·����Ϊ�����ʱʱ��ȡ�ȳ�������ֻ���ϵ�·��
    T_ave(Nc)=mean(T);
    %--------------------����ȫ�ֵ���Ϣ����----------------------------
    Delta_Tau=zeros(n,n);    %��ʼʱ��Ϣ��Ϊn*n��0����
    for i=1:m
        for j=1:(sum(Tabu(i,:)~=0,2)-1)  %size�����i��·���Ľڵ���
            Delta_Tau(Tabu(i,j),Tabu(i,j+1))=Delta_Tau(Tabu(i,j),Tabu(i,j+1))+Q/T(i);
            %�����Ͼ�����·����i��j���ϵ���Ϣ������
        end
    end
    Tau=rho.*Tau+Delta_Tau; %������Ϣ�ػӷ������º����Ϣ��
    %-----------------------------------------------------------------
    Tabu=zeros(m,n);
end
Pos=find(Tmin==min(Tmin)); %�ҵ����ʱ���λ�ã������ظ�ʱ��������ֵ�Ϊ׼
P_best=path(Pos(1),:) %���������������·��
T_best=Tmin(Pos(1)) %���������������ʱ��
plot(T_ave); %��ÿ��ƽ��ʱ���ͼ��
title('����ƽ��ʱ��')     
end

