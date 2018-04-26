function[X CompleteTime endtime]=MPDAdecode4X4qiongjuExp(R,T)
tic
Com=[];         %%存储每一次的完成时间
XX=[];
D=[1 2 3 4;1 2 4 3;1 3 2 4;1 3 4 2;1 4 2 3;1 4 3 2;2 1 3 4;2 1 4 3;2 3 1 4;2 3 4 1;2 4 1 3;2 4 3 1;3 1 2 4;3 1 4 2;3 2 1 4;3 2 4 1;3 4 1 2;3 4 2 1;4 1 2 3;4 1 3 2;4 2 1 3;4 2 3 1;4 3 2 1;4 3 1 2];
E=[];
p=1;
for i=1:24
    X(1,:)=D(i,:);
    for j=1:24
        X(2,:)=D(j,:);
        for k=1:24
            X(3,:)=D(k,:);
            for kk=1:24
                X(4,:)=D(kk,:);
                E{p}=X;
                p=p+1;
            end
        end
    end
end
for r=1:331776
    TT=zeros(150,8);   %TT=[rob task t到达 此刻任务点的变化率 此刻任务点状态值 预计任务完成时间 辅助 辅助]
    %  event=[rob_number task_number property time state];  %%property: 0表示到达，1表示完成，0时，t_path,1时，t
    %event=zeros(100,5);
    % load datacodewithdecode
    %load data3V3
    %load data3V3
    % X=[1 2 3;2 3 1;3 1 2];
    % R=[1 2 1 0.7;3 1 1 0.5;5 2 1 0.6];
    % T=[7 8 0.2 5;5 9 0.3 6;10 12 0.3 4];
    %XX=[1 1 2 2 3 3;2 2 3 3 1 1];
    u=4;
    z=4;
    X=E{r};
    L=exp(200);
    t=zeros(u,z);      %t=[t11 t12 t13;t21 t22 t23];
    ttt_path=zeros(u,z);
    tt_path=zeros(u,z);%%[ttp11 ttp12 ttp13;ttp21 ttp22 ttp23];  %%从机器人的初始位置去相应的任务点
    t_path=zeros(u,z);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%除11和21，均为从上一个任务点去下一个任务点，即机器人的位置在更新
    t_com=zeros(u,z);%%[tc11 tc12 tc13;tc21 tc22 tc23];
    
    Y=ones(2,z);
    for i=1:z
        Y(1,i)=i;
    end
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
    %% 计算初始到达时间矩阵t_path和完成时间矩阵 t
    for i=1:u
        a=0;
        RR=R;
        for j=1:z
            % time_rob(i,X(i,j))=sqrt((T(X(i,j),1)-RR(i,1))^2+(T(X(i,j),2)-RR(i,2))^2)/R(i,3)+a;
            %  t_path(i,X(i,j))=sqrt((T(X(i,j),1)-R(i,1))^2+(T(X(i,j),2)-R(i,2))^2)/R(i,3)+a;
            t_path(i,j)=sqrt((T(X(i,j),1)-RR(i,1))^2+(T(X(i,j),2)-RR(i,2))^2)/R(i,3)+a;
            %  property(i,2*j-1)=0;
            RR(i,1)=T(X(i,j),1);     %%接下来从上一个任务点出发去下一个任务点
            RR(i,2)=T(X(i,j),2);
            if(R(i,4)<=T(X(i,j),3))
                t_com(i,j)=inf;
            else
                t_com(i,j)=(log(10*T(X(i,j),4))+T(X(i,j),3)*t_path(i,j))/(R(i,4)-T(X(i,j),3));  %%以降到0.1为准
            end
            t(i,j)=t_path(i,j)+t_com(i,j);
            a=t(i,j);
        end
    end
    %%
    for i=1:u                  %%本循环的目的是得到当前最小的到达时间元素
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
    %%当前事件加入TT
    TT(n,1)=m(1);      %%机器人编号
    TT(n,2)=X(m(1),B(m(1)));    %%任务点编号
    TT(n,3)=s(1);                     %%到达时间
    TT(n,4)=T(X(m(1),B(m(1))),3)-R(m(1),4);         %%当前任务合成变化率
    if(abs(TT(n,4))<10^(-7))
        TT(n,4)=0;
    end
    TT(n,5)=log(10*T(X(m(1),B(m(1))),4))+T(X(m(1),B(m(1))),3)*TT(n,3);    %%当前状态值
    if(TT(n,4)>=0)
        TT(n,6)=inf;           %%完成时间
    else
        TT(n,6)=-TT(n,5)/TT(n,4)+TT(n,3);         %%完成时间
    end
    TT(n,7)=R(m(1),4);     %%此任务点周围聚集的机器人能力之和
    TT(n,8)=B(m(1));         %%辅助
    %% 译码
    flag1=0;
    while(flag1==0&&TT(n,3)~=L&&TT(n,3)~=inf)
        y=0;            %%flag1=0表示t_path中元素全不为-1，即还未完成译码工作
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
        for i=1:u        %%已被分配的元素完成时间置为-1
            for j=1:z
                if(i==m(1)&&j==B(m(1)))
                    t_path(i,j)=-1;
                end
            end
        end
        %   %测试
        %    t_path
        
        for i=1:u                  %%本循环的目的是得到当前最小的到达时间元素
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
        
        %测试
        %     A
        
        [s,m]=sort(A);
        %%当前事件加入TT
        TT(n,1)=m(1);      %%机器人编号
        TT(n,2)=X(m(1),B(m(1)));    %%任务点编号
        TT(n,3)=s(1);                     %%到达时间
        TT(n,4)=T(X(m(1),B(m(1))),3)-R(m(1),4);         %%当前任务合成变化率
        if(abs(TT(n,4))<10^(-7))
            TT(n,4)=0;
        end
        TT(n,5)=log(10*T(X(m(1),B(m(1))),4))+T(X(m(1),B(m(1))),3)*TT(n,3);    %%当前状态值
        if(TT(n,4)>=0)
            TT(n,6)=inf;           %%完成时间
        else
            TT(n,6)=-TT(n,5)/TT(n,4)+TT(n,3);         %%完成时间
        end
        TT(n,7)=R(m(1),4);     %%此任务点周围聚集的机器人能力之和
        TT(n,8)=B(m(1));         %%辅助
        
        
        %测试
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
            if(f>1)      %%当前事件对应的任务点与之前某次事件对应的任务点相同
                %             if(TT(n,3)<TT(i,6)||(TT(n,3)==TT(i,6)&&TT(i,6)==inf))    %%当前事件对应的到达该任务点的时间小于之前事件对应的该任务的完成时间，说明两者会产生合作
                TT(n,4)=TT(n,4)-TT(C(f),7);  %%迭代更新TT
                if(abs(TT(n,4))<10^(-7))
                    TT(n,4)=0;
                end
                TT(n,5)=TT(C(f),5)+TT(C(f),4)*(TT(n,3)-TT(C(f),3));
                if(TT(n,4)>=0)       %TT(n,4)表示当前任务点的合成变化率
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
                %                   C(f)=i;       %%C储存的是与当前任务点相同的之前事件的编号的集合
                %                   p=size(C,2);
                for ii=1:u
                    for q=1:f
                        if(ii==TT(C(q),1))
                            a=0;
                            RR=R;
                            for j=1:z            %%产生合作，更新到达时间矩阵和完成时间矩阵
                                t_path(ii,j)=sqrt((T(X(ii,j),1)-RR(ii,1))^2+(T(X(ii,j),2)-RR(ii,2))^2)/R(ii,3)+a;
                                %  property(i,2*j-1)=0;
                                RR(ii,1)=T(X(ii,j),1);     %%接下来从上一个任务点出发去下一个任务点
                                RR(ii,2)=T(X(ii,j),2);
                                if(R(ii,4)<=T(X(ii,j),3))
                                    t_com(ii,j)=inf;
                                else
                                    t_com(ii,j)=(log(10*T(X(ii,j),4))+T(X(ii,j),3)*t_path(ii,j))/(R(ii,4)-T(X(ii,j),3));  %%以降到0.1为准
                                end
                                t(ii,j)=t_path(ii,j)+t_com(ii,j);
                                %                                 if(ii==m(1)&&j==B(m(1)))
                                %                                    t(ii,j)=TT(n,6);
                                %                                 end
                                if(ii==TT(C(q),1)&&j==TT(C(q),8)) %%合作的机器人完成相同任务的完成时间相同
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
            if(flag==1)    %%%跳过当前任务点
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
                    for j=B(m(1)):z              %%更新时间矩阵
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
                            RR(m(1),1)=T(X(m(1),j),1);     %%接下来从上一个任务点出发去下一个任务点
                            RR(m(1),2)=T(X(m(1),j),2);
                            if(R(m(1),4)<=T(X(m(1),j),3))
                                t_com(m(1),j)=inf;
                            else
                                t_com(m(1),j)=(log(10*T(X(m(1),j),4))+T(X(m(1),j),3)*t_path(m(1),X(m(1),j)))/(R(m(1),4)-T(X(m(1),j),3));  %%以降到0.1为准
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
                    n=n-1;       %跳过当前事件，说明当前事件不会被执行，编号减一
                end
                %              end
            end
            %  end
            %       for k=1:(n-1)   %%更新TT
            %           if(TT(n,2)==TT(k,2))
            %                TT(k,6)=TT(n,6);
            %           end
            %       end
        end
        %   n=n+1;
        
        
        %测试
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
    % if(CompleteTime==inf)
    %    fprintf('The code is unvalid !\n')
    % else
    %    fprintf('The completion time of all the task is %f.\n',CompleteTime)
    % end
    Com(r)=CompleteTime;
    XX{r}=X;
end
[CompleteTime,~]=min(Com);
XXX=[];
om=1;
for i=1:331776
    if(Com(i)==CompleteTime)
        XXX{om}=XX{i};
        X=XXX{om};
        om=om+1;
    end
end
fprintf('The completion time of all the task is %f.\n',CompleteTime)
toc
endtime=toc;

