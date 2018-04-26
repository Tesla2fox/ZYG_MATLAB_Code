function[timeline]=shiyanNR4x4EXP(R,T)
%%记录的完成时间/运行时间序列
%%序列顺序为：{OC,MM,Mm,DSMM,DsMm,NsMm,Exhaustive,Random}
timeline=zeros(8,2);
%%随机生成
fprintf('******************OC************************* \n')
[X comtime end_time]=MPDAdecodeExpOC(R,T);
X
timeline(1,1)=comtime;
timeline(1,2)=end_time;
fprintf('*****************MM************************** \n')
[X comtime end_time]=MPDAdecodeminmax(R,T);
X
timeline(2,1)=comtime;
timeline(2,2)=end_time;
fprintf('*****************Mm************************** \n')
[X comtime end_time]=MPDAdecodeminmin(R,T);
X
timeline(3,1)=comtime;
timeline(3,2)=end_time;
fprintf('*****************DSMM************************** \n')
[X comtime end_time]=MPDAdecodeDynamicSumMinmax(R,T);
X
timeline(4,1)=comtime;
timeline(4,2)=end_time;
fprintf('*******************DSMm************************ \n')
[X comtime end_time]=MPDAdecodeDynamicSumMinmin(R,T);
X
timeline(5,1)=comtime;
timeline(5,2)=end_time;
fprintf('********************NsMm*********************** \n')
[X comtime end_time]=MPDAdecodeDynamicNsumMinmax(R,T);
X
timeline(6,1)=comtime;
timeline(6,2)=end_time;
fprintf('********************Exhaustive*********************** \n')
[X comtime end_time]=MPDAdecode4X4qiongjuExp(R,T);
X
timeline(7,1)=comtime;
timeline(7,2)=end_time;
fprintf('*********************Random********************** \n')
[X comtime end_time]=MPDAdecode_randomExp0919(R,T);
X
timeline(8,1)=comtime;
timeline(8,2)=end_time;

 

