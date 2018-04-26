%u=3;z=3;  %%u=rob_number,z=task_number
%function[encodeMat]=MPDAcodeExp(RobInfoMat,TaskInfoMat)
format short g

clear all;
 RobInfoMat=[1 2 1 0.2;
     3 1 1 0.3;
     5 2 1 0.4];
 TaskInfoMat= [7 8 0.15 5;5 9 0.25 6;10 12 0.12 4];

agentNum=size(RobInfoMat,1);
taskNum=size(TaskInfoMat,1);
AgentComTimeMat=zeros(agentNum,taskNum);      %t=[t11 t12 t13;t21 t22 t23];
Z=zeros(1,taskNum);  %%用来存储当前各个任务点的完成时间（已分配的算出得来）
%Q=zeros(u,z);%%[ttp11 ttp12 ttp13;ttp21 ttp22 ttp23];  %%从机器人的初始位置去相应的任务点
AgentArrTimeMat=zeros(agentNum,taskNum);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%除11和21，均为从上一个任务点去下一个任务点，即机器人的位置在更新
AgentEctTimeMat=zeros(agentNum,taskNum);%%[tc11 tc12 tc13;tc21 tc22 tc23];
encodeMat=zeros(agentNum,taskNum);
V=ones(agentNum,taskNum);
PredRatioMat=zeros(agentNum,taskNum);
M=zeros(agentNum,1);
D=zeros(agentNum,1);
sa=zeros(agentNum,1);
L=zeros(1,taskNum);
S3=zeros(1,taskNum);
S4=zeros(1,taskNum);
sb=zeros(1,taskNum);
S1=zeros(agentNum,taskNum);
S2=zeros(agentNum,taskNum);
Q=agentNum*taskNum;
H=10;       
omega=H/10;     %%ω
Ta=zeros(agentNum,taskNum);
fa=zeros(agentNum,taskNum); %%fa(i,j)=1时，表示i机器人放弃j任务点
%% 计算每个机器人单独去每个点的完成时间
for i=1:agentNum
    a=0;
    for j=1:taskNum
        AgentArrTimeMat(i,j)=sqrt((TaskInfoMat(j,1)-RobInfoMat(i,1))^2+(TaskInfoMat(j,2)-RobInfoMat(i,2))^2)/RobInfoMat(i,3);
        if(RobInfoMat(i,4)<=TaskInfoMat(j,3))
            AgentEctTimeMat(i,j)=inf;
        else
            AgentEctTimeMat(i,j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(i,j))/(RobInfoMat(i,4)-TaskInfoMat(j,3));  %%以降到0.1为准
        end
        AgentComTimeMat(i,j)=AgentArrTimeMat(i,j)+AgentEctTimeMat(i,j);
    end    
end

x=1;
flag=0;
for i=1:agentNum
    for j=1:taskNum
        PredRatioMat(i,j)=TaskInfoMat(j,3)-RobInfoMat(i,4);
    end
end
maxPredRatio=max(max(PredRatioMat));    %%βmax
%% 每个机器人到达任务点后对应一个任务点合成变化率，由于可能产生合作，故这些变化率可能各不相同
for i=1:agentNum           
    for j=1:taskNum
        Ta(i,j)=TaskInfoMat(j,3); 
    end
end
%%

while(flag==0)
    
    %判断启发式规则是否完成
    %此处可以改进
    y=0;            %%flag=0表示X中元素全不为0，即已被填满
    for i=1:agentNum
        for j=1:taskNum
            if(encodeMat(i,j)~=0)
               y=y+1;
            end
        end
    end
    if(y==(agentNum*taskNum))
        flag=1;
    end
    %测试
%     y
    %%
    maxArrTime=max(max(AgentArrTimeMat));     %%
    for i=1:agentNum
        for j=1:taskNum
           %极端情况：omega=1,先能力，后到达时间
           %极端情况：omega=0,先到达时间，后能力
            S1(i,j)=omega*(PredRatioMat(i,j)/maxPredRatio)+(1-omega)*(AgentArrTimeMat(i,j)/maxArrTime);  %%规则R1
            S2(i,j)=(1-omega)*(PredRatioMat(i,j)/maxPredRatio)+omega*(AgentArrTimeMat(i,j)/maxArrTime);  %%规则R2 
        end
    end
    
%%
    V=(AgentComTimeMat==inf);      %%有元素为无穷时，置1，非无穷数置0
    [q,w]=min(AgentComTimeMat);      %列最小
    [e,d]=max(q);      %行最大
    for k=1:taskNum
        D=(AgentComTimeMat(:,k)==inf);
        if(q(k)==inf)     %%q(k)=inf意味着第k列的t全为无穷和NaN
            if(sum(D)>1)
%                 D=D.*ones(u,1);
%                 for ii=1:u
%                     if(D(ii,1)==0)
%                        D(ii,1)=NaN;
%                     end
%                 end
                M=V(:,k).*S1(:,k);   %%取出第k列的S1
%                 M=M.*D;    
%                 for i=1:u
%                     if(M(i,1)==0)
%                         M(i,1)=NaN;       %%已经被分配过的元素设为NaN，不参与比较和计算
%                     end
%                 end
               [g,h]=min(M);
               sa=(M==g);
               if(sum(sa)>1)                %%按照S1出来的元素有多个
                   sa=sa.*ones(agentNum,1);
                   for ii=1:agentNum
                       if(sa(ii,1)==0)
                           sa(ii,1)=NaN;
                       end
                   end
                   M=sa.*S2(:,k);            %%按S2继续筛选
                   [~,h]=min(M); 
               end
               w(k)=h;          %%第k列的最优执行机器人编号为w(k)
            end
        end
    end
    L=(q==inf);   %%衡量q有几个无穷，大于1个得考虑选哪个无穷
    for k=1:taskNum
        S3(1,k)=S1(w(k),k);   %所有列取出的值对应的S1值
        S4(1,k)=S2(w(k),k);   %所有列取出的值对应的S2值
    end
    L=L.*ones(1,taskNum);
    if(sum(L)>1)
       for ii=1:taskNum
           if(L(1,ii)==0)
              L(1,ii)=nan;
           end
       end
       S3=S3.*L;
       [gg,hh]=max(S3);     %行取S1值大一点的，因为比较紧急
       sb=(S3==gg);
       if(sum(sb)>1)     %%按S1取出的元素有多个
           sb=sb.*ones(1,taskNum);
           for ii=1:taskNum
               if(sb(1,ii)==0)
                  sb(1,ii)=NaN;
               end
           end
           S4=S4.*sb;
           [~,hh]=max(S4);
       end
       d=hh;       %%最优分配元素所处的列的编号，则最优分配元素对应的位置在（w(d),d）
    end
    %%
% 测试
%     flag
%     t_path
%     t
%     q
%     w
%     e
%     d
%     vvv=t(w(d),d);
%     vvv
    for i=1:taskNum
        if(encodeMat(w(d),i)==0)
            encodeMat(w(d),i)=d;       %取一个注入X(编码矩阵)
            break;
        end
    end
    chsAgID = w(d);
    chsTskID = d;
    
%测试    
%     x=x+1;
%      X
%     t(w(d),d)
   % t(w(d),d)=inf;     %使用完置为无穷
   
    if(AgentComTimeMat(w(d),d)==inf)     %%当最优元素列均为无穷，继续在此列中筛选直到相应任务可以被完成
        B=[];
        [~,bb]=sort(S1(:,d));  %%此处按照S1排序，S1越小，排序越前，越先被分配
%         v=bb(1)-w(d);
%         bb
        for i=1:agentNum
            if(bb(i)==w(d))
                v=i;
                break;
            end
        end
        if(v>1)          %%由于(w(d),d)先被分配，将其在bb中的编号提到最前，目的是方便于程序
           for i=v:-1:2
               vv=bb(i);
               bb(i)=bb(i-1);
               bb(i-1)=vv;
           end
        end
        lamda=TaskInfoMat(d,3)-RobInfoMat(w(d),4);
        for i=2:agentNum
            lamda=lamda-RobInfoMat(bb(i),4); %%分配一个，计算一次合成变化率
            if(abs(lamda)<10^(-7))
               lamda=0; 
            end
            for j=1:taskNum
                if(encodeMat(bb(i),j)==0)
                    encodeMat(bb(i),j)=d;       %取一个注入X(编码矩阵)
                    break;
                end
            end
            AgentComTimeMat(bb(i),d)=NaN;
            if(lamda<0)        %%当合成变化率第一次小于0，当前列分配结束
                cc=i;          %%cc是合作完成当前任务的机器人的个数
                break;
            end
        end
        for i=1:cc                 
            B(i)=AgentArrTimeMat(bb(i),d);
        end
        [F,f]=sort(B);           %% 对合作的机器人的到达时间排序，方便计算任务的完成时间
        P=TaskInfoMat(d,3)*F(1); 
        lamda=TaskInfoMat(d,3);
        for i=2:cc
            lamda=lamda-RobInfoMat(bb(f(i-1)),4);
            if(abs(lamda)<10^(-7))
               lamda=0; 
            end
            P=P+lamda*(F(i)-F(i-1));
        end
        lamda=lamda-RobInfoMat(bb(f(cc)),4);
        tcom=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(cc);        %%合作执行当前任务的完成时间
        Z(d)=tcom;              %%
        s=TaskInfoMat(d,3);
%        T(d,3)=lamda;  %%更新变化率，循环使用
        if(cc<agentNum)
        for i=(cc+1):agentNum                      %%分配完之后，需要更新此列中剩下元素对应的完成时间
            if(AgentArrTimeMat(bb(i),d)<tcom)        
                %%其他元素对应的到达时间小于前面得出的该任务完成时间，说明此机器人也会参与该任务的执行，更新完成时间
                B(cc+1)=AgentArrTimeMat(bb(i),d);
                [F,f]=sort(B);
                P=s*F(1); 
                lamda=s;
                for k=2:(cc+1)
                    lamda=lamda-RobInfoMat(bb(f(k-1)),4);
                    if(abs(lamda)<10^(-7))
                       lamda=0; 
                    end
                    P=P+lamda*(F(k)-F(k-1));
                end
                lamda=lamda-RobInfoMat(bb(f(cc+1)),4);
                Ta(bb(i),d)=lamda;                   %%%%更新Ta
                AgentComTimeMat(bb(i),d)=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(cc+1);
            end
            if(AgentArrTimeMat(bb(i),d)>=tcom)           %%其他元素对应的到达时间大于前面得出的该任务完成时间，说明此元素对应的机器人无需到此任务点
               AgentComTimeMat(bb(i),d)=NaN; 
               fa(bb(i),d)=1;
               for j=taskNum:-1:1
                   if(encodeMat(bb(i),j)==0)
                      encodeMat(bb(i),j)=d;
                      break;
                   end
               end
            end 
        end
        end
%         测试
%         t
%         tcom
        for i=1:cc                 %%更新已分配元素所在行的完成时间
            for j=1:taskNum
                if(j~=d)
                   if(isnan(AgentComTimeMat(bb(i),j))==1)    %已被分配的不考虑
                      continue; 
                   end
                end
                if(j~=d)    
                   AgentArrTimeMat(bb(i),j)=sqrt((TaskInfoMat(j,1)-TaskInfoMat(d,1))^2+(TaskInfoMat(j,2)-TaskInfoMat(d,2))^2)/RobInfoMat(bb(i),3)+tcom;
                   Ba=[];      
                   Ca=[];
                   m=0; 
                   n=0;
                   for r=1:agentNum
                       if(r==bb(i))
                           continue;
                       end
                       if(isnan(AgentComTimeMat(r,j))==1)
                           n=n+1;
                       end
                   end
                   for r=1:agentNum
                       if(r==bb(i))
                           continue;
                       end
                       if(isnan(AgentComTimeMat(r,j))==1&&fa(r,j)==0)
                           m=m+1;
                           Ba(m)=AgentArrTimeMat(r,j);
                           Ca(m)=r;
                       end
                   end
                   if(n==0)
                      if(RobInfoMat(bb(i),4)>TaskInfoMat(j,3))
                          AgentEctTimeMat(bb(i),j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(w(d),j))/(RobInfoMat(bb(i),4)-TaskInfoMat(j,3));  %%以降到0.1为准
                      else
                          AgentEctTimeMat(bb(i),j)=inf;
                      end
                          AgentComTimeMat(bb(i),j)=AgentArrTimeMat(bb(i),j)+AgentEctTimeMat(bb(i),j); 
                   end
                   if(m>0)                                        %%只要无穷列更新完，除了该列任务的第一次分配，其他的行更新不是跳过就是合作
                     if(AgentArrTimeMat(bb(i),j)<Z(j))
                        Ba(m+1)=AgentArrTimeMat(bb(i),j);
                        Ca(m+1)=bb(i);
                       [F,f]=sort(Ba);
                       P=TaskInfoMat(j,3)*F(1); 
                       lamda=TaskInfoMat(j,3);
                       for k=2:(m+1)
                           lamda=lamda-RobInfoMat(Ca(f(k-1)),4);
                           if(abs(lamda)<10^(-7))
                              lamda=0; 
                           end
                           P=P+lamda*(F(k)-F(k-1));
                       end
                       lamda=lamda-RobInfoMat(Ca(f(m+1)),4);
                       Ta(bb(i),j)=lamda;                   %%%%更新Ta
                       AgentComTimeMat(bb(i),j)=-(log(10*TaskInfoMat(j,4))+P)/lamda + F(m+1);  
                     end
                     if(AgentArrTimeMat(bb(i),j)>=Z(j))
                        AgentComTimeMat(bb(i),j)=NaN; 
                        fa(bb(i),j)=1;
                        for k=taskNum:-1:1
                            if(encodeMat(bb(i),k)==0)
                               encodeMat(bb(i),k)=j;
                               break;
                            end
                        end 
                     end
                   end
                end
            end
        end
    end
%     t_path
%     t
%     X
    Ba=[];      
    Ca=[];
    m=1;
    %%
    if(AgentComTimeMat(w(d),d)~=inf)       %%当前最优元素列均为有限数
        Ba(1)=AgentArrTimeMat(w(d),d);
        Ca(1)=w(d);
        for i=1:agentNum
            if(isnan(AgentComTimeMat(i,d))==1&&fa(i,d)==0)
                m=m+1;
                Ba(m)=AgentArrTimeMat(i,d);
                Ca(m)=i;
            end
        end
%         m
        if(m>1)
           [F,f]=sort(Ba);
           P=TaskInfoMat(d,3)*F(1); 
           lamda=TaskInfoMat(d,3);
           for k=2:m
               lamda=lamda-RobInfoMat(Ca(f(k-1)),4);
               if(abs(lamda)<10^(-7))
                  lamda=0; 
               end
               P=P+lamda*(F(k)-F(k-1));
           end
           lamda=lamda-RobInfoMat(Ca(f(m)),4);
           Z(d)=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(m); 
           for i=1:agentNum
               if(i==w(d))
                  continue;
               else
                   if(isnan(AgentComTimeMat(i,d))==1)
                      continue; 
                   end
               end
               if(AgentArrTimeMat(i,d)<AgentComTimeMat(w(d),d))         %%需要合作，更新此列中其他元素的完成时间
                   Ba(m+1)=AgentArrTimeMat(i,d);
                   Ca(m+1)=i;
                   [F,f]=sort(Ba);
                    P=TaskInfoMat(d,3)*F(1); 
                   lamda=TaskInfoMat(d,3);
                   for k=2:(m+1)
                       lamda=lamda-RobInfoMat(Ca(f(k-1)),4);
                       if(abs(lamda)<10^(-7))
                          lamda=0; 
                       end
                       P=P+lamda*(F(k)-F(k-1));
                   end
                   lamda=lamda-RobInfoMat(Ca(f(m+1)),4);
                   Ta(i,d)=lamda;                   %%%%更新Ta
                   AgentComTimeMat(i,d)=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(m+1); 
               end
               if(AgentArrTimeMat(i,d)>=AgentComTimeMat(w(d),d)) %%不需合作，跳过此任务
                   AgentComTimeMat(i,d)=NaN; 
                   fa(i,d)=1;
                   for j=taskNum:-1:1
                       if(encodeMat(i,j)==0)
                          encodeMat(i,j)=d;
                          break;
                       end
                   end
               end
           end
        end
        
        if(m==1) 
            Z(d)=AgentComTimeMat(w(d),d);
        for i=1:agentNum
            if(i==w(d))
               continue;
            else
               if(isnan(AgentComTimeMat(i,d))==1)
                   continue;
               end
            end
            if(AgentArrTimeMat(i,d)<AgentComTimeMat(w(d),d))         %%需要合作，更新此列中其他元素的完成时间
                if(AgentArrTimeMat(i,d)<AgentArrTimeMat(w(d),d))
%                     P=T(d,4)*exp(T(d,3)*t_path(i,d))*exp((T(d,3)-R(i,4))*(t_path(w(d),d)-t_path(i,d)));
%                     t_com(i,d)=log(10*P)/(R(i,4)+R(w(d),4)-T(d,3));
                    P=TaskInfoMat(d,3)*AgentArrTimeMat(i,d)+(TaskInfoMat(d,3)-RobInfoMat(i,4))*(AgentArrTimeMat(w(d),d)-AgentArrTimeMat(i,d));
                    AgentEctTimeMat(i,d)=(log(10*TaskInfoMat(d,4))+P)/(RobInfoMat(i,4)+RobInfoMat(w(d),4)-TaskInfoMat(d,3));
                else
%                     P=T(d,4)*exp(T(d,3)*t_path(w(d),d))*exp((T(d,3)-R(w(d),4))*(t_path(i,d)-t_path(w(d),d)));
%                     t_com(i,d)=log(10*P)/(R(i,4)+R(w(d),4)-T(d,3));
                    P=TaskInfoMat(d,3)*AgentArrTimeMat(w(d),d)+(TaskInfoMat(d,3)-RobInfoMat(w(d),4))*(AgentArrTimeMat(i,d)-AgentArrTimeMat(w(d),d));
                    AgentEctTimeMat(i,d)=(log(10*TaskInfoMat(d,4))+P)/(RobInfoMat(i,4)+RobInfoMat(w(d),4)-TaskInfoMat(d,3));
                end
                if(AgentEctTimeMat(i,d)<=0)     %%说明任务当前无法被完成
                   AgentComTimeMat(i,d)=inf;
                end
                AgentComTimeMat(i,d)=AgentArrTimeMat(i,d)+AgentEctTimeMat(i,d);
                Ta(i,d)=TaskInfoMat(d,3)-RobInfoMat(w(d),4)-RobInfoMat(i,4);
            end
            if(AgentArrTimeMat(i,d)>=AgentComTimeMat(w(d),d)) %%不需合作，跳过此任务
               AgentComTimeMat(i,d)=NaN;
               fa(i,d)=1;
               for j=taskNum:-1:1
                   if(encodeMat(i,j)==0)
                      encodeMat(i,j)=d;
                      break;
                   end
               end
               x=x+1;
            end
%             t
        end
        end
        
%       [~,hhh]=min(t(:,d));
        for j=1:taskNum
            if(j~=d)
               if(isnan(AgentComTimeMat(w(d),j))==1)
                   continue; 
               end
            end
            if(j~=d)         %%更新当前行其他元素的完成时间
               AgentArrTimeMat(w(d),j)=sqrt((TaskInfoMat(j,1)-TaskInfoMat(d,1))^2+(TaskInfoMat(j,2)-TaskInfoMat(d,2))^2)/RobInfoMat(w(d),3)+Z(d);  %%预计t_path(w(d),j)
               Ba=[];      
               Ca=[];
               m=0; 
               n=0;
               for i=1:agentNum
                   if(i==w(d))
                       continue;
                   end
                   if(isnan(AgentComTimeMat(i,j))==1)
                       n=n+1;
                   end
               end
               for i=1:agentNum
                   if(i==w(d))
                       continue;
                   end
                   if(isnan(AgentComTimeMat(i,j))==1&&fa(i,j)==0)
                       m=m+1;
                       Ba(m)=AgentArrTimeMat(i,j);
                       Ca(m)=i;
                   end
               end
               if(n==0)
                  if(RobInfoMat(w(d),4)>TaskInfoMat(j,3))
                      AgentEctTimeMat(w(d),j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(w(d),j))/(RobInfoMat(w(d),4)-TaskInfoMat(j,3));  %%以降到0.1为准
                  else
                      AgentEctTimeMat(w(d),j)=inf;
                  end
                      AgentComTimeMat(w(d),j)=AgentArrTimeMat(w(d),j)+AgentEctTimeMat(w(d),j); 
               end
               if(m>0)                                        %%只要无穷列更新完，除了该列任务的第一次分配，其他的行更新不是跳过就是合作
                 if(AgentArrTimeMat(w(d),j)<Z(j))
                   Ba(m+1)=AgentArrTimeMat(w(d),j);
                   Ca(m+1)=w(d);
                   [F,f]=sort(Ba);
                   P=TaskInfoMat(j,3)*F(1); 
                   lamda=TaskInfoMat(j,3);
                   for k=2:(m+1)
                       lamda=lamda-RobInfoMat(Ca(f(k-1)),4);
                       if(abs(lamda)<10^(-7))
                           lamda=0; 
                       end
                       P=P+lamda*(F(k)-F(k-1));
                   end
                   lamda=lamda-RobInfoMat(Ca(f(m+1)),4);
                   Ta(w(d),j)=lamda;                   %%%%更新Ta
                   AgentComTimeMat(w(d),j)=-(log(10*TaskInfoMat(j,4))+P)/lamda + F(m+1);  
                 end
                 if(AgentArrTimeMat(w(d),j)>=Z(j))
                    AgentComTimeMat(w(d),j)=NaN; 
                    fa(w(d),j)=1;
                    for k=taskNum:-1:1
                        if(encodeMat(w(d),k)==0)
                           encodeMat(w(d),k)=j;
                           break;
                        end
                    end 
                 end
               end
            end
        end
    end
%测试
%      t(w(d),d)
%      t
%      X
%      Z
    AgentComTimeMat(w(d),d)=NaN;  %%当前被分配元素置为NaN，之后不参与比较和计算
 %   V(W(d),d)=1;
end
% X
