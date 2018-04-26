%function[TT]=MPDAdecodeExp(X,R,T)
tic
eveMat=zeros(15000,8);   %TT=[rob task t到达 此刻任务点的变化率 此刻任务点状态值 预计任务完成时间 辅助 辅助]
%  event=[rob_number task_number property time state];  %%property: 0表示到达，1表示完成，0时，t_path,1时，t 
%event=zeros(100,5);
% load datacodewithdecode
%load data3V3
%load data3V3



 encodeMat=[1 2 3 4 ;2 3 1 4 ;3 1 2 4 ];
 RobInfoMat=[1 2 1 0.7;3 1 1 0.5;5 2 1 0.6];
 TaskInfoMat=[7 8 0.2 5;
                        5 9 0.3 6;
                       10 12 0.3 4;
                       7 1 0.5 1];
 
%XX=[1 1 2 2 3 3;2 2 3 3 1 1];
U=size(encodeMat);
agentNum=U(1);
taskNum=U(2);
LargeNum=exp(200);
t=zeros(agentNum,taskNum);      %t=[t11 t12 t13;t21 t22 t23];
ttt_path=zeros(agentNum,taskNum);
tt_path=zeros(agentNum,taskNum);%%[ttp11 ttp12 ttp13;ttp21 ttp22 ttp23];  %%从机器人的初始位置去相应的任务点
AgentArrTimeMat=zeros(agentNum,taskNum);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%除11和21，均为从上一个任务点去下一个任务点，即机器人的位置在更新
AgentComTimeMat=zeros(agentNum,taskNum);%%[tc11 tc12 tc13;tc21 tc22 tc23];

Y=ones(2,taskNum);
for i=1:taskNum
    Y(1,i)=i;
end
A=[];
B=[];
C=[];
pf=0;
% time_rob=zeros(3,6);
% property=zeros(3,6);
eveIndex=1;
m=0;
findMinArrTimeBoolean=0;
format short g
%% 计算初始到达时间矩阵t_path和完成时间矩阵 t
for i=1:agentNum
    a=0;
    RR=RobInfoMat;
    for j=1:taskNum
       % time_rob(i,X(i,j))=sqrt((T(X(i,j),1)-RR(i,1))^2+(T(X(i,j),2)-RR(i,2))^2)/R(i,3)+a;
     %  t_path(i,X(i,j))=sqrt((T(X(i,j),1)-R(i,1))^2+(T(X(i,j),2)-R(i,2))^2)/R(i,3)+a;
        AgentArrTimeMat(i,j)=sqrt((TaskInfoMat(encodeMat(i,j),1)-RR(i,1))^2+(TaskInfoMat(encodeMat(i,j),2)-RR(i,2))^2)/RobInfoMat(i,3)+a;
      %  property(i,2*j-1)=0;
        RR(i,1)=TaskInfoMat(encodeMat(i,j),1);     %%接下来从上一个任务点出发去下一个任务点
        RR(i,2)=TaskInfoMat(encodeMat(i,j),2);
    if(RobInfoMat(i,4)<=TaskInfoMat(encodeMat(i,j),3))
        AgentComTimeMat(i,j)=inf;
    else
        AgentComTimeMat(i,j)=(log(10*TaskInfoMat(encodeMat(i,j),4))+TaskInfoMat(encodeMat(i,j),3)*AgentArrTimeMat(i,j))/(RobInfoMat(i,4)-TaskInfoMat(encodeMat(i,j),3));  %%以降到0.1为准
    end
        t(i,j)=AgentArrTimeMat(i,j)+AgentComTimeMat(i,j);
        a=t(i,j);
    end
end

%%
fprintf('wtf');
%%
 for i=1:agentNum                  %%本循环的目的是得到当前最小的到达时间元素
      findMinArrTimeBoolean=0;
      for j=1:taskNum   
          if(AgentArrTimeMat(i,j)~=-1)
              B(i)=j;
              A(i)=AgentArrTimeMat(i,j);
              findMinArrTimeBoolean=1;
              break;
          end 
      end
      if(findMinArrTimeBoolean==0)
         A(i)=LargeNum; 
      end
 end
   [s,m]=sort(A);       
  %%当前事件加入TT
  eveMat(eveIndex,1)=m(1);      %%机器人编号
  eveMat(eveIndex,2)=encodeMat(m(1),B(m(1)));    %%任务点编号
  eveMat(eveIndex,3)=s(1);                     %%到达时间
  eveMat(eveIndex,4)=TaskInfoMat(encodeMat(m(1),B(m(1))),3)-RobInfoMat(m(1),4);         %%当前任务合成变化率
  if(abs(eveMat(eveIndex,4))<10^(-7))
     eveMat(eveIndex,4)=0; 
  end
  eveMat(eveIndex,5)=log(10*TaskInfoMat(encodeMat(m(1),B(m(1))),4))+TaskInfoMat(encodeMat(m(1),B(m(1))),3)*eveMat(eveIndex,3);    %%当前状态值
  if(eveMat(eveIndex,4)>=0)
      eveMat(eveIndex,6)=inf;           %%完成时间
  else
      eveMat(eveIndex,6)=-eveMat(eveIndex,5)/eveMat(eveIndex,4)+eveMat(eveIndex,3);         %%完成时间
  end
  eveMat(eveIndex,7)=RobInfoMat(m(1),4);     %%此任务点周围聚集的机器人能力之和
  eveMat(eveIndex,8)=B(m(1));         %%辅助
 %% 译码
flag1=0;
while(flag1==0&&eveMat(eveIndex,3)~=LargeNum&&eveMat(eveIndex,3)~=inf)
    y=0;            %%flag1=0表示t_path中元素全不为-1，即还未完成译码工作
    %此处判断是否完成全部译码工作，若完成全部译码工作则
    for i=1:agentNum
        for j=1:taskNum
            if(AgentArrTimeMat(i,j)==-1)
               y=y+1;
            end
        end
    end
    if(y==(agentNum*taskNum))
        flag1=1;
    end
    %没有必要遍历。
    
    eveIndex=eveIndex+1;
%     g=TT(n,3);
%     g

%此处仍然可以进行优化。？？？
    for i=1:agentNum        %%已被分配的元素完成时间置为-1
      for j=1:taskNum
          if(i==m(1)&&j==B(m(1)))
             AgentArrTimeMat(i,j)=-1;
          end
      end 
    end
    
    
%   %测试
%    t_path
  for i=1:agentNum                  %%本循环的目的是得到当前最小的到达时间元素
      findMinArrTimeBoolean=0;
      for j=1:taskNum   
          if(AgentArrTimeMat(i,j)~=-1)
              B(i)=j;
              A(i)=AgentArrTimeMat(i,j);
              findMinArrTimeBoolean=1;
              break;
          end 
      end
      if(findMinArrTimeBoolean==0)
         A(i)=LargeNum; 
      end
  end
 % if(size(A)==0)
  
  %测试
%     A
  [s,m]=sort(A);       
  %%当前事件加入TT
  eveMat(eveIndex,1)=m(1);      %%机器人编号
  eveMat(eveIndex,2)=encodeMat(m(1),B(m(1)));    %%任务点编号
  eveMat(eveIndex,3)=s(1);                     %%到达时间
  eveMat(eveIndex,4)=TaskInfoMat(encodeMat(m(1),B(m(1))),3)-RobInfoMat(m(1),4);         %%当前任务合成变化率
  if(abs(eveMat(eveIndex,4))<10^(-7))
     eveMat(eveIndex,4)=0; 
  end
  eveMat(eveIndex,5)=log(10*TaskInfoMat(encodeMat(m(1),B(m(1))),4))+TaskInfoMat(encodeMat(m(1),B(m(1))),3)*eveMat(eveIndex,3);    %%当前状态值
  if(eveMat(eveIndex,4)>=0)
      eveMat(eveIndex,6)=inf;           %%完成时间
  else
      eveMat(eveIndex,6)=-eveMat(eveIndex,5)/eveMat(eveIndex,4)+eveMat(eveIndex,3);         %%完成时间
  end
  eveMat(eveIndex,7)=RobInfoMat(m(1),4);     %%此任务点周围聚集的机器人能力之和
  eveMat(eveIndex,8)=B(m(1));         %%辅助

  if(eveMat(eveIndex,3)==inf)     %%无法完成
     break; 
  end
  %测试
%   s        
% TT        
% m(1)
flag=0;

  if(eveIndex>=2)
      C=[];
      f=1;
      C(f)=eveIndex;
      for i=1:eveIndex-1
          if(eveMat(eveIndex,2)==eveMat(i,2))
              if(eveMat(eveIndex,3)>eveMat(i,6)||(eveMat(eveIndex,3)==eveMat(i,6)&&eveMat(eveIndex,3)~=inf))
                  flag=1;
                  break;
              end
              if(eveMat(eveIndex,3)<eveMat(i,6)||(eveMat(eveIndex,3)==eveMat(i,6)&&eveMat(i,6)==inf))
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
                  eveMat(eveIndex,4)=eveMat(eveIndex,4)-eveMat(C(f),7);  %%迭代更新TT
                  if(abs(eveMat(eveIndex,4))<10^(-7))
                      eveMat(eveIndex,4)=0; 
                  end
                  eveMat(eveIndex,5)=eveMat(C(f),5)+eveMat(C(f),4)*(eveMat(eveIndex,3)-eveMat(C(f),3));
                  if(eveMat(eveIndex,4)>=0)       %TT(n,4)表示当前任务点的合成变化率
                      eveMat(eveIndex,6)=inf;
                  else
                      eveMat(eveIndex,6)=-eveMat(eveIndex,5)/eveMat(eveIndex,4)+eveMat(eveIndex,3);
                  end
%                  TT(C(f),6)=TT(n,6);
                  for i=2:f
                      eveMat(C(i),6)=eveMat(eveIndex,6);
                  end
                  eveMat(eveIndex,7)=eveMat(C(f),7)+eveMat(eveIndex,7);
                  for k=1:agentNum
                      for j=1:taskNum
                          ttt_path(k,j)=AgentArrTimeMat(k,j);
                      end
                  end
%                   f=f+1;
%                   C(f)=i;       %%C储存的是与当前任务点相同的之前事件的编号的集合
%                   p=size(C,2);  
                  for ii=1:agentNum
                      for q=1:f
                      if(ii==eveMat(C(q),1))   
                      a=0;
                      RR=RobInfoMat;
                      for j=1:taskNum            %%产生合作，更新到达时间矩阵和完成时间矩阵
                          AgentArrTimeMat(ii,j)=sqrt((TaskInfoMat(encodeMat(ii,j),1)-RR(ii,1))^2+(TaskInfoMat(encodeMat(ii,j),2)-RR(ii,2))^2)/RobInfoMat(ii,3)+a;
                           %  property(i,2*j-1)=0;
                          RR(ii,1)=TaskInfoMat(encodeMat(ii,j),1);     %%接下来从上一个任务点出发去下一个任务点
                          RR(ii,2)=TaskInfoMat(encodeMat(ii,j),2);
                          if(RobInfoMat(ii,4)<=TaskInfoMat(encodeMat(ii,j),3))
                                AgentComTimeMat(ii,j)=inf;
                          else
                                AgentComTimeMat(ii,j)=(log(10*TaskInfoMat(encodeMat(ii,j),4))+TaskInfoMat(encodeMat(ii,j),3)*AgentArrTimeMat(ii,j))/(RobInfoMat(ii,4)-TaskInfoMat(encodeMat(ii,j),3));  %%以降到0.1为准
                          end
                                t(ii,j)=AgentArrTimeMat(ii,j)+AgentComTimeMat(ii,j);
%                                 if(ii==m(1)&&j==B(m(1)))
%                                    t(ii,j)=TT(n,6); 
%                                 end
                                if(ii==eveMat(C(q),1)&&j==eveMat(C(q),8)) %%合作的机器人完成相同任务的完成时间相同
                                   t(ii,j)=eveMat(eveIndex,6); 
                                end
                                a=t(ii,j);
                      end
                      end
                      end
                  end
                  
                  for k=1:agentNum
                      for j=1:taskNum
                          if(ttt_path(k,j)==-1)
                               AgentArrTimeMat(k,j)=-1;
                          end
                      end
                  end
%              end
           end
              if(flag==1)    %%%跳过当前任务点
                  if(eveMat(eveIndex,3)==LargeNum)
                      eveMat(eveIndex,2)=nan;
                      break;
                  else
                  for j=1:taskNum
                      tt_path(m(1),j)=AgentArrTimeMat(m(1),j);
                  end
                  if(B(m(1))==1)
                      a=0;
                      RR=RobInfoMat;
                  else
                      a=t(m(1),B(m(1))-1);  
                      RR=RobInfoMat;
                      RR(m(1),1)=TaskInfoMat(encodeMat(m(1),B(m(1))-1),1);    
                      RR(m(1),2)=TaskInfoMat(encodeMat(m(1),B(m(1))-1),2);
                  end
                  for j=B(m(1)):taskNum              %%更新时间矩阵
                      if(j==B(m(1)))
                          AgentArrTimeMat(m(1),j)=-1;
                          if(B(m(1))==1)
                               t(m(1),j)=0;
                          else
                              t(m(1),j)=t(m(1),j-1);
                          end
                          continue;
                      else
                          AgentArrTimeMat(m(1),j)=sqrt((TaskInfoMat(encodeMat(m(1),j),1)-RR(m(1),1))^2+(TaskInfoMat(encodeMat(m(1),j),2)-RR(m(1),2))^2)/RobInfoMat(m(1),3)+a;
                          RR(m(1),1)=TaskInfoMat(encodeMat(m(1),j),1);     %%接下来从上一个任务点出发去下一个任务点
                          RR(m(1),2)=TaskInfoMat(encodeMat(m(1),j),2);
                          if(RobInfoMat(m(1),4)<=TaskInfoMat(encodeMat(m(1),j),3))
                              AgentComTimeMat(m(1),j)=inf;
                          else
                              AgentComTimeMat(m(1),j)=(log(10*TaskInfoMat(encodeMat(m(1),j),4))+TaskInfoMat(encodeMat(m(1),j),3)*AgentArrTimeMat(m(1),encodeMat(m(1),j)))/(RobInfoMat(m(1),4)-TaskInfoMat(encodeMat(m(1),j),3));  %%以降到0.1为准
                          end
                          t(m(1),j)=AgentArrTimeMat(m(1),j)+AgentComTimeMat(m(1),j);
                          a=t(m(1),j);
                      end
                  end
                  for j=1:taskNum
                      if(tt_path(m(1),j)==-1)
                          AgentArrTimeMat(m(1),j)=-1;
                      end
                  end
                  eveIndex=eveIndex-1;       %跳过当前事件，说明当前事件不会被执行，编号减一
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
TTT=zeros(eveIndex,1);
for i=1:eveIndex-1
    TTT(i)=eveMat(i,6);
end
[CompleteTime,ff]=max(TTT);
eveMat=eveMat(1:eveIndex-1,:);
eveMat=eveMat(:,1:7);
eveMat(:,[3,5])=eveMat(:,[5,3]);
eveMat(:,[4,6])=eveMat(:,[6,4]);
eveMat(:,[4,5])=eveMat(:,[5,4]);
Th=eveMat;
Th(:,3)=[];
Th(:,5)=[];
Th(:,5)=[];
if(CompleteTime==inf)
   fprintf('The code is unvalid !\n')
else
   fprintf('The completion time of all the task is %f.\n',CompleteTime)
end
toc

