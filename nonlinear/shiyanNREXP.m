%function[timeline]=shiyanNREXP(R,T)
%%记录的完成时间/运行时间序列
%%序列顺序为：{OC,MM,Mm,DSMM,DsMm,NsMm,Exhaustive,Random}
timeline=zeros(8,2);
R=[1 2 1 0.2;3 1 1 0.2;5 2 1 0.6];
T=[7 8 0.9 5;5 9 0.9 6;10 12 0.9 4];

%%随机生成
fprintf('******************OC************************* \n')
%[X comtime end_time]=MPDAdecodeExpOC(R,T);  %%时间复杂度为O(n^2)
%timeline(1,1)=comtime;
%timeline(1,2)=end_time;
fprintf('*****************MM************************** \n')
[X comtime end_time]=MPDAdecodeminmax(R,T);  %%时间复杂度为O(n^2)
timeline(2,1)=comtime;
timeline(2,2)=end_time;
fprintf('*****************Mm************************** \n')
[X comtime end_time]=MPDAdecodeminmin(R,T);  %%时间复杂度为O(n^2)
timeline(3,1)=comtime;
timeline(3,2)=end_time;
fprintf('*****************DSMM************************** \n')
[X comtime end_time]=MPDAdecodeDynamicSumMinmax(R,T);  %%时间复杂度O(n^3)
timeline(4,1)=comtime;
timeline(4,2)=end_time;
fprintf('*******************DSMm************************ \n')
[X comtime end_time]=MPDAdecodeDynamicSumMinmin(R,T); %%时间复杂度O(n^3)
timeline(5,1)=comtime;
timeline(5,2)=end_time;
fprintf('********************NsMm*********************** \n')
[X comtime end_time]=MPDAdecodeDynamicNsumMinmax(R,T); %%时间复杂度O(n^3)
timeline(6,1)=comtime;
timeline(6,2)=end_time;
% fprintf('********************Exhaustive*********************** \n')
% [X comtime end_time]=MPDAdecode5X3qiongjuExp(R,T);
% X
timeline(7,1)=inf;
timeline(7,2)=inf;
fprintf('*********************Random********************** \n')
[X comtime end_time]=MPDAdecode_randomExp0919(R,T);
timeline(8,1)=comtime;
timeline(8,2)=end_time;

 

