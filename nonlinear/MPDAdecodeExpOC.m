function[X CompleteTime end_time TT]=MPDAdecodeExpOC(R,T)   %%ʱ�临�Ӷȣ� O(n^2)(n=u*z)
tic
TT=zeros(15000,8);   %TT=[rob task t���� �˿������ı仯�� �˿������״ֵ̬ Ԥ���������ʱ�� ���� ����]
X=MPDAcodeExp(R,T);  %%ʱ�临�Ӷȣ�O(n^2)(n=u*z)(uΪ�����˸�����zΪ��������)
%% ����ʱ�临�Ӷ�O(n^2)
%  ����ʱ�临�Ӷ�Ϊ O(n^2)
U=size(X); 
u=U(1);
z=U(2);
L=exp(200);
t=zeros(u,z);      %t=[t11 t12 t13;t21 t22 t23];
ttt_path=zeros(u,z);
tt_path=zeros(u,z);%%[ttp11 ttp12 ttp13;ttp21 ttp22 ttp23];  %%�ӻ����˵ĳ�ʼλ��ȥ��Ӧ�������
t_path=zeros(u,z);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%��11��21����Ϊ����һ�������ȥ��һ������㣬�������˵�λ���ڸ���
t_com=zeros(u,z);%%[tc11 tc12 tc13;tc21 tc22 tc23];
% Y=ones(2,z);
% for i=1:z
%     Y(1,i)=i;
% end
A=[];
B=[];
C=[];
pf=0;
% time_rob=zeros(3,6);
% property=zeros(3,6);
n=1;
m=0;
sf=0;
format short g
%% �����ʼ����ʱ�����t_path�����ʱ����� t
for i=1:u
    a=0;
    RR=R;
    for j=1:z
        % time_rob(i,X(i,j))=sqrt((T(X(i,j),1)-RR(i,1))^2+(T(X(i,j),2)-RR(i,2))^2)/R(i,3)+a;
        %  t_path(i,X(i,j))=sqrt((T(X(i,j),1)-R(i,1))^2+(T(X(i,j),2)-R(i,2))^2)/R(i,3)+a;
        t_path(i,j)=sqrt((T(X(i,j),1)-RR(i,1))^2+(T(X(i,j),2)-RR(i,2))^2)/R(i,3)+a;
        %  property(i,2*j-1)=0;
        RR(i,1)=T(X(i,j),1);     %%����������һ����������ȥ��һ�������
        RR(i,2)=T(X(i,j),2);
        if(R(i,4)<=T(X(i,j),3))
            t_com(i,j)=inf;
        else
            t_com(i,j)=(log(10*T(X(i,j),4))+T(X(i,j),3)*t_path(i,j))/(R(i,4)-T(X(i,j),3));  %%�Խ���0.1Ϊ׼
        end
        t(i,j)=t_path(i,j)+t_com(i,j);
        a=t(i,j);
    end
end
%%
for i=1:u                  %%��ѭ����Ŀ���ǵõ���ǰ��С�ĵ���ʱ��Ԫ��
    sf=0;
    for j=1:z
        if(t_path(i,j)~=-1)
            B(i)=j;
            A(i)=t_path(i,j);
            sf=1;
            break;
        end
    end
    if(sf==0)
        A(i)=L;
    end
end
[s,m]=sort(A);
%%��ǰ�¼�����TT
TT(n,1)=m(1);      %%�����˱��
TT(n,2)=X(m(1),B(m(1)));    %%�������
TT(n,3)=s(1);                     %%����ʱ��
TT(n,4)=T(X(m(1),B(m(1))),3)-R(m(1),4);         %%��ǰ����ϳɱ仯��
if(abs(TT(n,4))<10^(-7))
    TT(n,4)=0;
end
TT(n,5)=log(10*T(X(m(1),B(m(1))),4))+T(X(m(1),B(m(1))),3)*TT(n,3);    %%��ǰ״ֵ̬
if(TT(n,4)>=0)
    TT(n,6)=inf;           %%���ʱ��
else
    TT(n,6)=-TT(n,5)/TT(n,4)+TT(n,3);         %%���ʱ��
end
TT(n,7)=R(m(1),4);     %%���������Χ�ۼ��Ļ���������֮��
TT(n,8)=B(m(1));         %%����
%% ����
flag1=0;
while(flag1==0&&TT(n,3)~=L&&TT(n,3)~=inf)
    y=0;            %%flag1=0��ʾt_path��Ԫ��ȫ��Ϊ-1������δ������빤��
    for i=1:u
        for j=1:z
            if(t_path(i,j)==-1)
                y=y+1;
            end
        end
    end
    if(y==(u*z))
        flag1=1;
    end
    n=n+1;
    %     g=TT(n,3);
    %     g
    for i=1:u        %%�ѱ������Ԫ�����ʱ����Ϊ-1
        for j=1:z
            if(i==m(1)&&j==B(m(1)))
                t_path(i,j)=-1;
            end
        end
    end
    %   %����
    %    t_path
    
    for i=1:u                  %%��ѭ����Ŀ���ǵõ���ǰ��С�ĵ���ʱ��Ԫ��
        sf=0;
        for j=1:z
            if(t_path(i,j)~=-1)
                B(i)=j;
                A(i)=t_path(i,j);
                sf=1;
                break;
            end
        end
        if(sf==0)
            A(i)=L;
        end
    end
    % if(size(A)==0)
    
    %����
    %     A
    
    [s,m]=sort(A);
    %%��ǰ�¼�����TT
    TT(n,1)=m(1);      %%�����˱��
    TT(n,2)=X(m(1),B(m(1)));    %%�������
    TT(n,3)=s(1);                     %%����ʱ��
    TT(n,4)=T(X(m(1),B(m(1))),3)-R(m(1),4);         %%��ǰ����ϳɱ仯��
    if(abs(TT(n,4))<10^(-7))
        TT(n,4)=0;
    end
    TT(n,5)=log(10*T(X(m(1),B(m(1))),4))+T(X(m(1),B(m(1))),3)*TT(n,3);    %%��ǰ״ֵ̬
    if(TT(n,4)>=0)
        TT(n,6)=inf;           %%���ʱ��
    else
        TT(n,6)=-TT(n,5)/TT(n,4)+TT(n,3);         %%���ʱ��
    end
    TT(n,7)=R(m(1),4);     %%���������Χ�ۼ��Ļ���������֮��
    TT(n,8)=B(m(1));         %%����
    if(TT(n,3)==inf)     %%�޷����
        break; 
    end
    
    %����
    %   s
    % TT
    % m(1)
    flag=0;
    if(n>=2)
        C=[];
        f=1;
        C(f)=n;
        for i=1:n-1
            if(TT(n,2)==TT(i,2))
                if(TT(n,3)>TT(i,6)||(TT(n,3)==TT(i,6)&&TT(n,3)~=inf))
                    flag=1;
                    break;
                end
                if(TT(n,3)<TT(i,6)||(TT(n,3)==TT(i,6)&&TT(i,6)==inf))
                    f=f+1;
                    C(f)=i;
                end
            end
        end
        %       f
        %       flag
        %     p=size(C,2);
        %     for i=1:(n-1)
        if(f>1)      %%��ǰ�¼���Ӧ���������֮ǰĳ���¼���Ӧ���������ͬ
            %             if(TT(n,3)<TT(i,6)||(TT(n,3)==TT(i,6)&&TT(i,6)==inf))    %%��ǰ�¼���Ӧ�ĵ����������ʱ��С��֮ǰ�¼���Ӧ�ĸ���������ʱ�䣬˵�����߻��������
            TT(n,4)=TT(n,4)-TT(C(f),7);  %%��������TT
            if(abs(TT(n,4))<10^(-7))
                TT(n,4)=0;
            end
            TT(n,5)=TT(C(f),5)+TT(C(f),4)*(TT(n,3)-TT(C(f),3));
            if(TT(n,4)>=0)       %TT(n,4)��ʾ��ǰ�����ĺϳɱ仯��
                TT(n,6)=inf;
            else
                TT(n,6)=-TT(n,5)/TT(n,4)+TT(n,3);
            end
            %                  TT(C(f),6)=TT(n,6);
            for i=2:f
                TT(C(i),6)=TT(n,6);
            end
            TT(n,7)=TT(C(f),7)+TT(n,7);
            for k=1:u
                for j=1:z
                    ttt_path(k,j)=t_path(k,j);
                end
            end
            %                   f=f+1;
            %                   C(f)=i;       %%C��������뵱ǰ�������ͬ��֮ǰ�¼��ı�ŵļ���
            %                   p=size(C,2);
            for ii=1:u
                for q=1:f
                    if(ii==TT(C(q),1))
                        a=0;
                        RR=R;
                        for j=1:z            %%�������������µ���ʱ���������ʱ�����
                            t_path(ii,j)=sqrt((T(X(ii,j),1)-RR(ii,1))^2+(T(X(ii,j),2)-RR(ii,2))^2)/R(ii,3)+a;
                            %  property(i,2*j-1)=0;
                            RR(ii,1)=T(X(ii,j),1);     %%����������һ����������ȥ��һ�������
                            RR(ii,2)=T(X(ii,j),2);
                            if(R(ii,4)<=T(X(ii,j),3))
                                t_com(ii,j)=inf;
                            else
                                t_com(ii,j)=(log(10*T(X(ii,j),4))+T(X(ii,j),3)*t_path(ii,j))/(R(ii,4)-T(X(ii,j),3));  %%�Խ���0.1Ϊ׼
                            end
                            t(ii,j)=t_path(ii,j)+t_com(ii,j);
                            %                                 if(ii==m(1)&&j==B(m(1)))
                            %                                    t(ii,j)=TT(n,6);
                            %                                 end
                            if(ii==TT(C(q),1)&&j==TT(C(q),8)) %%�����Ļ����������ͬ��������ʱ����ͬ
                                t(ii,j)=TT(n,6);
                            end
                            a=t(ii,j);
                        end
                    end
                end
            end
            
            for k=1:u
                for j=1:z
                    if(ttt_path(k,j)==-1)
                        t_path(k,j)=-1;
                    end
                end
            end
            %              end
        end
        if(flag==1)    %%%������ǰ�����
            if(TT(n,3)==L)
                TT(n,2)=nan;
                break;
            else
                for j=1:z
                    tt_path(m(1),j)=t_path(m(1),j);
                end
                if(B(m(1))==1)
                    a=0;
                    RR=R;
                else
                    a=t(m(1),B(m(1))-1);
                    RR=R;
                    RR(m(1),1)=T(X(m(1),B(m(1))-1),1);
                    RR(m(1),2)=T(X(m(1),B(m(1))-1),2);
                end
                for j=B(m(1)):z              %%����ʱ�����
                    if(j==B(m(1)))
                        t_path(m(1),j)=-1;
                        if(B(m(1))==1)
                            t(m(1),j)=0;
                        else
                            t(m(1),j)=t(m(1),j-1);
                        end
                        continue;
                    else
                        t_path(m(1),j)=sqrt((T(X(m(1),j),1)-RR(m(1),1))^2+(T(X(m(1),j),2)-RR(m(1),2))^2)/R(m(1),3)+a;
                        RR(m(1),1)=T(X(m(1),j),1);     %%����������һ����������ȥ��һ�������
                        RR(m(1),2)=T(X(m(1),j),2);
                        if(R(m(1),4)<=T(X(m(1),j),3))
                            t_com(m(1),j)=inf;
                        else
                            t_com(m(1),j)=(log(10*T(X(m(1),j),4))+T(X(m(1),j),3)*t_path(m(1),X(m(1),j)))/(R(m(1),4)-T(X(m(1),j),3));  %%�Խ���0.1Ϊ׼
                        end
                        t(m(1),j)=t_path(m(1),j)+t_com(m(1),j);
                        a=t(m(1),j);
                    end
                end
                for j=1:z
                    if(tt_path(m(1),j)==-1)
                        t_path(m(1),j)=-1;
                    end
                end
                n=n-1;       %������ǰ�¼���˵����ǰ�¼����ᱻִ�У���ż�һ
            end
            %              end
        end
        %  end
        %       for k=1:(n-1)   %%����TT
        %           if(TT(n,2)==TT(k,2))
        %                TT(k,6)=TT(n,6);
        %           end
        %       end
    end
    %   n=n+1;
    
    
    %����
    %   TT
    %   flag1
    %   g=TT(n,3);
    %     g
end
TTT=zeros(n,1);
for i=1:n-1
    TTT(i)=TT(i,6);
end
[CompleteTime,ff]=max(TTT);
TT=TT(1:n-1,:);
TT=TT(:,1:7);
TT(:,[3,5])=TT(:,[5,3]);
TT(:,[4,6])=TT(:,[6,4]);
TT(:,[4,5])=TT(:,[5,4]);
Th=TT;
Th(:,3)=[];
Th(:,5)=[];
Th(:,5)=[];
if(CompleteTime==inf)
    fprintf('The code is unvalid !\n')
else
    fprintf('The completion time of all the task is %f.\n',CompleteTime)
end
toc
end_time=toc;

