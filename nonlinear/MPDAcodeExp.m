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
Z=zeros(1,taskNum);  %%�����洢��ǰ�������������ʱ�䣨�ѷ�������������
%Q=zeros(u,z);%%[ttp11 ttp12 ttp13;ttp21 ttp22 ttp23];  %%�ӻ����˵ĳ�ʼλ��ȥ��Ӧ�������
AgentArrTimeMat=zeros(agentNum,taskNum);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%��11��21����Ϊ����һ�������ȥ��һ������㣬�������˵�λ���ڸ���
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
omega=H/10;     %%��
Ta=zeros(agentNum,taskNum);
fa=zeros(agentNum,taskNum); %%fa(i,j)=1ʱ����ʾi�����˷���j�����
%% ����ÿ�������˵���ȥÿ��������ʱ��
for i=1:agentNum
    a=0;
    for j=1:taskNum
        AgentArrTimeMat(i,j)=sqrt((TaskInfoMat(j,1)-RobInfoMat(i,1))^2+(TaskInfoMat(j,2)-RobInfoMat(i,2))^2)/RobInfoMat(i,3);
        if(RobInfoMat(i,4)<=TaskInfoMat(j,3))
            AgentEctTimeMat(i,j)=inf;
        else
            AgentEctTimeMat(i,j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(i,j))/(RobInfoMat(i,4)-TaskInfoMat(j,3));  %%�Խ���0.1Ϊ׼
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
maxPredRatio=max(max(PredRatioMat));    %%��max
%% ÿ�������˵����������Ӧһ�������ϳɱ仯�ʣ����ڿ��ܲ�������������Щ�仯�ʿ��ܸ�����ͬ
for i=1:agentNum           
    for j=1:taskNum
        Ta(i,j)=TaskInfoMat(j,3); 
    end
end
%%

while(flag==0)
    
    %�ж�����ʽ�����Ƿ����
    %�˴����ԸĽ�
    y=0;            %%flag=0��ʾX��Ԫ��ȫ��Ϊ0�����ѱ�����
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
    %����
%     y
    %%
    maxArrTime=max(max(AgentArrTimeMat));     %%
    for i=1:agentNum
        for j=1:taskNum
           %���������omega=1,���������󵽴�ʱ��
           %���������omega=0,�ȵ���ʱ�䣬������
            S1(i,j)=omega*(PredRatioMat(i,j)/maxPredRatio)+(1-omega)*(AgentArrTimeMat(i,j)/maxArrTime);  %%����R1
            S2(i,j)=(1-omega)*(PredRatioMat(i,j)/maxPredRatio)+omega*(AgentArrTimeMat(i,j)/maxArrTime);  %%����R2 
        end
    end
    
%%
    V=(AgentComTimeMat==inf);      %%��Ԫ��Ϊ����ʱ����1������������0
    [q,w]=min(AgentComTimeMat);      %����С
    [e,d]=max(q);      %�����
    for k=1:taskNum
        D=(AgentComTimeMat(:,k)==inf);
        if(q(k)==inf)     %%q(k)=inf��ζ�ŵ�k�е�tȫΪ�����NaN
            if(sum(D)>1)
%                 D=D.*ones(u,1);
%                 for ii=1:u
%                     if(D(ii,1)==0)
%                        D(ii,1)=NaN;
%                     end
%                 end
                M=V(:,k).*S1(:,k);   %%ȡ����k�е�S1
%                 M=M.*D;    
%                 for i=1:u
%                     if(M(i,1)==0)
%                         M(i,1)=NaN;       %%�Ѿ����������Ԫ����ΪNaN��������ȽϺͼ���
%                     end
%                 end
               [g,h]=min(M);
               sa=(M==g);
               if(sum(sa)>1)                %%����S1������Ԫ���ж��
                   sa=sa.*ones(agentNum,1);
                   for ii=1:agentNum
                       if(sa(ii,1)==0)
                           sa(ii,1)=NaN;
                       end
                   end
                   M=sa.*S2(:,k);            %%��S2����ɸѡ
                   [~,h]=min(M); 
               end
               w(k)=h;          %%��k�е�����ִ�л����˱��Ϊw(k)
            end
        end
    end
    L=(q==inf);   %%����q�м����������1���ÿ���ѡ�ĸ�����
    for k=1:taskNum
        S3(1,k)=S1(w(k),k);   %������ȡ����ֵ��Ӧ��S1ֵ
        S4(1,k)=S2(w(k),k);   %������ȡ����ֵ��Ӧ��S2ֵ
    end
    L=L.*ones(1,taskNum);
    if(sum(L)>1)
       for ii=1:taskNum
           if(L(1,ii)==0)
              L(1,ii)=nan;
           end
       end
       S3=S3.*L;
       [gg,hh]=max(S3);     %��ȡS1ֵ��һ��ģ���Ϊ�ȽϽ���
       sb=(S3==gg);
       if(sum(sb)>1)     %%��S1ȡ����Ԫ���ж��
           sb=sb.*ones(1,taskNum);
           for ii=1:taskNum
               if(sb(1,ii)==0)
                  sb(1,ii)=NaN;
               end
           end
           S4=S4.*sb;
           [~,hh]=max(S4);
       end
       d=hh;       %%���ŷ���Ԫ���������еı�ţ������ŷ���Ԫ�ض�Ӧ��λ���ڣ�w(d),d��
    end
    %%
% ����
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
            encodeMat(w(d),i)=d;       %ȡһ��ע��X(�������)
            break;
        end
    end
    chsAgID = w(d);
    chsTskID = d;
    
%����    
%     x=x+1;
%      X
%     t(w(d),d)
   % t(w(d),d)=inf;     %ʹ������Ϊ����
   
    if(AgentComTimeMat(w(d),d)==inf)     %%������Ԫ���о�Ϊ��������ڴ�����ɸѡֱ����Ӧ������Ա����
        B=[];
        [~,bb]=sort(S1(:,d));  %%�˴�����S1����S1ԽС������Խǰ��Խ�ȱ�����
%         v=bb(1)-w(d);
%         bb
        for i=1:agentNum
            if(bb(i)==w(d))
                v=i;
                break;
            end
        end
        if(v>1)          %%����(w(d),d)�ȱ����䣬������bb�еı���ᵽ��ǰ��Ŀ���Ƿ����ڳ���
           for i=v:-1:2
               vv=bb(i);
               bb(i)=bb(i-1);
               bb(i-1)=vv;
           end
        end
        lamda=TaskInfoMat(d,3)-RobInfoMat(w(d),4);
        for i=2:agentNum
            lamda=lamda-RobInfoMat(bb(i),4); %%����һ��������һ�κϳɱ仯��
            if(abs(lamda)<10^(-7))
               lamda=0; 
            end
            for j=1:taskNum
                if(encodeMat(bb(i),j)==0)
                    encodeMat(bb(i),j)=d;       %ȡһ��ע��X(�������)
                    break;
                end
            end
            AgentComTimeMat(bb(i),d)=NaN;
            if(lamda<0)        %%���ϳɱ仯�ʵ�һ��С��0����ǰ�з������
                cc=i;          %%cc�Ǻ�����ɵ�ǰ����Ļ����˵ĸ���
                break;
            end
        end
        for i=1:cc                 
            B(i)=AgentArrTimeMat(bb(i),d);
        end
        [F,f]=sort(B);           %% �Ժ����Ļ����˵ĵ���ʱ�����򣬷��������������ʱ��
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
        tcom=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(cc);        %%����ִ�е�ǰ��������ʱ��
        Z(d)=tcom;              %%
        s=TaskInfoMat(d,3);
%        T(d,3)=lamda;  %%���±仯�ʣ�ѭ��ʹ��
        if(cc<agentNum)
        for i=(cc+1):agentNum                      %%������֮����Ҫ���´�����ʣ��Ԫ�ض�Ӧ�����ʱ��
            if(AgentArrTimeMat(bb(i),d)<tcom)        
                %%����Ԫ�ض�Ӧ�ĵ���ʱ��С��ǰ��ó��ĸ��������ʱ�䣬˵���˻�����Ҳ�����������ִ�У��������ʱ��
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
                Ta(bb(i),d)=lamda;                   %%%%����Ta
                AgentComTimeMat(bb(i),d)=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(cc+1);
            end
            if(AgentArrTimeMat(bb(i),d)>=tcom)           %%����Ԫ�ض�Ӧ�ĵ���ʱ�����ǰ��ó��ĸ��������ʱ�䣬˵����Ԫ�ض�Ӧ�Ļ��������赽�������
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
%         ����
%         t
%         tcom
        for i=1:cc                 %%�����ѷ���Ԫ�������е����ʱ��
            for j=1:taskNum
                if(j~=d)
                   if(isnan(AgentComTimeMat(bb(i),j))==1)    %�ѱ�����Ĳ�����
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
                          AgentEctTimeMat(bb(i),j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(w(d),j))/(RobInfoMat(bb(i),4)-TaskInfoMat(j,3));  %%�Խ���0.1Ϊ׼
                      else
                          AgentEctTimeMat(bb(i),j)=inf;
                      end
                          AgentComTimeMat(bb(i),j)=AgentArrTimeMat(bb(i),j)+AgentEctTimeMat(bb(i),j); 
                   end
                   if(m>0)                                        %%ֻҪ�����и����꣬���˸�������ĵ�һ�η��䣬�������и��²����������Ǻ���
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
                       Ta(bb(i),j)=lamda;                   %%%%����Ta
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
    if(AgentComTimeMat(w(d),d)~=inf)       %%��ǰ����Ԫ���о�Ϊ������
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
               if(AgentArrTimeMat(i,d)<AgentComTimeMat(w(d),d))         %%��Ҫ���������´���������Ԫ�ص����ʱ��
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
                   Ta(i,d)=lamda;                   %%%%����Ta
                   AgentComTimeMat(i,d)=-(log(10*TaskInfoMat(d,4))+P)/lamda + F(m+1); 
               end
               if(AgentArrTimeMat(i,d)>=AgentComTimeMat(w(d),d)) %%�������������������
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
            if(AgentArrTimeMat(i,d)<AgentComTimeMat(w(d),d))         %%��Ҫ���������´���������Ԫ�ص����ʱ��
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
                if(AgentEctTimeMat(i,d)<=0)     %%˵������ǰ�޷������
                   AgentComTimeMat(i,d)=inf;
                end
                AgentComTimeMat(i,d)=AgentArrTimeMat(i,d)+AgentEctTimeMat(i,d);
                Ta(i,d)=TaskInfoMat(d,3)-RobInfoMat(w(d),4)-RobInfoMat(i,4);
            end
            if(AgentArrTimeMat(i,d)>=AgentComTimeMat(w(d),d)) %%�������������������
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
            if(j~=d)         %%���µ�ǰ������Ԫ�ص����ʱ��
               AgentArrTimeMat(w(d),j)=sqrt((TaskInfoMat(j,1)-TaskInfoMat(d,1))^2+(TaskInfoMat(j,2)-TaskInfoMat(d,2))^2)/RobInfoMat(w(d),3)+Z(d);  %%Ԥ��t_path(w(d),j)
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
                      AgentEctTimeMat(w(d),j)=(log(10*TaskInfoMat(j,4))+TaskInfoMat(j,3)*AgentArrTimeMat(w(d),j))/(RobInfoMat(w(d),4)-TaskInfoMat(j,3));  %%�Խ���0.1Ϊ׼
                  else
                      AgentEctTimeMat(w(d),j)=inf;
                  end
                      AgentComTimeMat(w(d),j)=AgentArrTimeMat(w(d),j)+AgentEctTimeMat(w(d),j); 
               end
               if(m>0)                                        %%ֻҪ�����и����꣬���˸�������ĵ�һ�η��䣬�������и��²����������Ǻ���
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
                   Ta(w(d),j)=lamda;                   %%%%����Ta
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
%����
%      t(w(d),d)
%      t
%      X
%      Z
    AgentComTimeMat(w(d),d)=NaN;  %%��ǰ������Ԫ����ΪNaN��֮�󲻲���ȽϺͼ���
 %   V(W(d),d)=1;
end
% X
