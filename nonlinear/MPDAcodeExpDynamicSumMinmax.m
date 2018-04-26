%u=3;z=3;  %%u=rob_number,z=task_number
%%SingleH3
function[XX]=MPDAcodeExpDynamicSumMinmax(R,T)
format short g
q=11;
for H=1:q
%     if(H==1)
%         HH=1.1;
%     else
%         if(H==11)
%             HH=10.9;
%         else
%             HH=H;
%         end
%     end
    u=size(R,1);
    z=size(T,1);
    Ta=T;
    Q=u*z;
    t=zeros(u,z);      %t=[t11 t12 t13;t21 t22 t23];
    E=zeros(u,z);
    t_path=zeros(u,z);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%��11��21����Ϊ����һ�������ȥ��һ������㣬�������˵�λ���ڸ���
    t_com=zeros(u,z);%%[tc11 tc12 tc13;tc21 tc22 tc23];
    X=zeros(u,z);
    omega=(H-1)/(q-1);     %%��
    fa=zeros(u,z); %%fa(i,j)=1ʱ����ʾi�����˷���j�����
    Z=zeros(1,z);  %%�����洢��ǰ�������������ʱ�䣨�ѷ�������������
    
    for i=1:u
        for j=1:z
            t_path(i,j)=sqrt((T(j,1)-R(i,1))^2+(T(j,2)-R(i,2))^2)/R(i,3);
            if(R(i,4)<=T(j,3))
                t_com(i,j)=inf;
            else
                t_com(i,j)=(log(10*T(j,4))+T(j,3)*t_path(i,j))/(R(i,4)-T(j,3));  %%�Խ���0.1Ϊ׼
            end
            t(i,j)=t_path(i,j)+t_com(i,j);
        end
    end
    flag=0;
    while(flag==0)
        test=X;
        y=0;            %%flag=0��ʾX��Ԫ��ȫ��Ϊ0�����ѱ�����
        for i=1:u
            for j=1:z
                if(X(i,j)~=0)
                    y=y+1;
                end
            end
        end
        if(y==Q)
            flag=1;
        end
        c=0;
        p=1;
        A=[];
        B=[];
        C=[];
        D=[];
        Pp=zeros(1,Q);
        for i=1:u
            for j=1:z
                if(isnan(t(i,j))==1)
                    A(p)=nan;
                    B(p)=-1;
                    Pp(p)=nan;
                    p=p+1;
                    c=c+1;
                else
                    A(p)=t_path(i,j);
                    B(p)=t(i,j);
                    p=p+1;
                end
            end
        end
        if(c==Q)
            break;
        end
        [e,x]=sort(A);
        [g,s]=sort(B,'descend');
        r=1;
        for i=1:(Q-c)
            C(i)=r;
            if(i==Q-c)
                break;
            end
            if(e(i+1)~=e(i))
                r=r+1;
            end
        end
        if(g(Q-c)==inf)          %%�������񵥸������˶��������  C��D�������Ҫ����ľ���
            B=[];
            p=1;
            for i=1:u
                for j=1:z
                    if(isnan(t(i,j))==1)
                        B(p)=-1;
                        p=p+1;
                    else
                        E(i,j)=R(i,4)+Ta(j,3);
                        B(p)=E(i,j);
                        p=p+1;
                    end
                end
            end
            [h,s]=sort(B,'descend');
            r=1;
            for i=1:Q-c
                D(i)=r;
                if(i==Q-c)
                    break;
                else
                    if(h(i+1)~=h(i))
                        r=r+1;
                    end
                end
            end
        end
        
        if(g(1)~=inf)
            if(g(1)==-1)
                break;
            else
                r=1;
                for i=1:Q-c
                    D(i)=r;
                    if(i==Q-c)
                        break;
                    else
                        if(g(i+1)~=g(i))
                            r=r+1;
                        end
                    end
                end
            end
        end
        
        if(g(Q-c)~=inf&&g(1)==inf)
            for i=2:Q-c
                if(g(i)~=inf)
                    fs=i;           %%f�ǵ�һ��������ɵ�R-T��
                    break;
                end
            end
            F=[];
            n=1;
            for i=1:fs-1
                v=ceil(s(i)/z);       %%��Ӧ�Ļ����˱��
                b=rem(s(i),z);        %%��Ӧ���������
                if(b==0)
                    b=z;
                end
                F(n)=R(v,4)+Ta(b,3);
                n=n+1;
            end
            [w,l]=sort(F,'descend');
            n=1;
            ss=s;
            for i=1:fs-1
                s(i)=ss(l(n));
                n=n+1;
            end
            r=1;
            for i=1:fs-1
                D(i)=r;
                if(i==fs-1)
                    break;
                else
                    if(w(l(i+1))~=w(l(i)))
                        r=r+1;
                    end
                end
            end
            r=r+1;
            for i=fs:Q-c
                D(i)=r;
                if(i==Q-c)
                    break;
                else
                    if(g(i+1)~=g(i))
                        r=r+1;
                    end
                end
            end
        end
        for i=1:Q-c
            for j=1:Q-c
                if(x(i)==s(j))
                    Pp(x(i))=omega*C(i)+(1-omega)*D(j);
                    break;
                end
            end
        end
        [~,hh]=min(Pp);
        v=ceil(hh/z);       %%��Ӧ�Ļ����˱��
        b=rem(hh,z);        %%��Ӧ���������
        if(b==0)
            b=z;
        end
        Ta(b,3)=Ta(b,3)-R(v,4);
        for k=1:z
            if(X(v,k)==0)
                X(v,k)=b;
                break;
            end
        end
        PP=Pp;
        flag1=0;
        dd=[];
        dd(1)=v;
        aa=2;
        amax=0;
        if(t(v,b)==inf)
            rr=2;
            N=[];
            M=[];
            N(1)=t_path(v,b);
            M(1)=v;
            lamda=T(b,3)-R(v,4);
            if(abs(lamda)<10^(-7))
                lamda=0;
            end
            while(flag1==0)
                for i=1:u
                    flag3=0;
                    for k=1:aa-1
                        if(i==dd(k))
                            flag3=1;
                            break;
                        end
                    end
                    if(flag3==1)
                        continue;
                    else
                        a=0;
                        for j=1:z
                            if(isnan(t(i,j))==1)
                                Pp((i-1)*z+j)=max(PP)+1;
                            end
                            a=a+Pp((i-1)*z+j);           %%%%��ͣ��Ƚ�
                        end
                        if(a>amax)
                            amax=a;
                            d=i;
                        end
                    end
                end
                dd(aa)=d;
                aa=aa+1;
                for k=1:z
                    if(X(d,k)==0)
                        X(d,k)=b;
                        break;
                    end
                end
                t(d,b)=nan;
                M(rr)=d;
                N(rr)=t_path(d,b);
                rr=rr+1;
                lamda=lamda-R(d,4);
                if(abs(lamda)<10^(-7))
                    lamda=0;
                end
                if(lamda<0)
                    flag1=1;
                end
            end
            [F,f]=sort(N);           %% �Ժ����Ļ����˵ĵ���ʱ�����򣬷��������������ʱ��
            P=T(b,3)*F(1);
            lamda=T(b,3);
            for i=2:(rr-1)
                lamda=lamda-R(M(f(i-1)),4);
                if(abs(lamda)<10^(-7))
                    lamda=0;
                end
                P=P+lamda*(F(i)-F(i-1));
            end
            lamda=lamda-R(M(f(rr-1)),4);
            tcom=-(log(10*T(b,4))+P)/lamda + F(rr-1);        %%����ִ�е�ǰ��������ʱ��
            Z(b)=tcom;              %%
            s=T(b,3);
            MM=M;
            if((rr-1)<u)
                for i=1:u                      %%������֮����Ҫ���´�����ʣ��Ԫ�ض�Ӧ�����ʱ��
                    flag2=0;
                    for j=1:rr-1
                        if(i==M(j))
                            flag2=1;
                            break;
                        end
                    end
                    if(flag2==0)
                        if(t_path(i,b)<tcom)         %%����Ԫ�ض�Ӧ�ĵ���ʱ��С��ǰ��ó��ĸ��������ʱ�䣬˵���˻�����Ҳ�����������ִ�У��������ʱ��
                            N(rr)=t_path(i,b);
                            M(rr)=i;
                            [F,f]=sort(N);
                            P=s*F(1);
                            lamda=s;
                            for k=2:rr
                                lamda=lamda-R(M(f(k-1)),4);
                                if(abs(lamda)<10^(-7))
                                    lamda=0;
                                end
                                P=P+lamda*(F(k)-F(k-1));
                            end
                            lamda=lamda-R(M(f(rr)),4);
                            Ta(i,b)=lamda;                   %%%%����Ta
                            t(i,b)=-(log(10*T(b,4))+P)/lamda + F(rr);
                        end
                        if(t_path(i,b)>=tcom)           %%����Ԫ�ض�Ӧ�ĵ���ʱ�����ǰ��ó��ĸ��������ʱ�䣬˵����Ԫ�ض�Ӧ�Ļ��������赽�������
                            t(i,b)=NaN;
                            fa(i,b)=1;
                            for j=z:-1:1
                                if(X(i,j)==0)
                                    X(i,j)=b;
                                    break;
                                end
                            end
                        end
                    end
                end
            end
            for i=1:rr-1                 %%�����ѷ���Ԫ�������е����ʱ��
                for j=1:z
                    if(j~=b)
                        if(isnan(t(MM(i),j))==1)    %�ѱ�����Ĳ�����
                            continue;
                        end
                    end
                    if(j~=b)
                        t_path(MM(i),j)=sqrt((T(j,1)-T(b,1))^2+(T(j,2)-T(b,2))^2)/R(MM(i),3)+tcom;
                        Ba=[];
                        Ca=[];
                        m=0;
                        n=0;
                        for r=1:u
                            if(r==MM(i))
                                continue;
                            end
                            if(isnan(t(r,j))==1)
                                n=n+1;
                            end
                        end
                        for r=1:u
                            if(r==MM(i))
                                continue;
                            end
                            if(isnan(t(r,j))==1&&fa(r,j)==0)
                                m=m+1;
                                Ba(m)=t_path(r,j);
                                Ca(m)=r;
                            end
                        end
                        if(n==0)
                            if(R(MM(i),4)>T(j,3))
                                t_com(MM(i),j)=(log(10*T(j,4))+T(j,3)*t_path(v,j))/(R(MM(i),4)-T(j,3));  %%�Խ���0.1Ϊ׼
                            else
                                t_com(MM(i),j)=inf;
                            end
                            t(MM(i),j)=t_path(MM(i),j)+t_com(MM(i),j);
                        end
                        if(m>0)                                        %%ֻҪ�����и����꣬���˸�������ĵ�һ�η��䣬�������и��²����������Ǻ���
                            if(t_path(MM(i),j)<Z(j))
                                Ba(m+1)=t_path(MM(i),j);
                                Ca(m+1)=MM(i);
                                [F,f]=sort(Ba);
                                P=T(j,3)*F(1);
                                lamda=T(j,3);
                                for k=2:(m+1)
                                    lamda=lamda-R(Ca(f(k-1)),4);
                                    if(abs(lamda)<10^(-7))
                                        lamda=0;
                                    end
                                    P=P+lamda*(F(k)-F(k-1));
                                end
                                lamda=lamda-R(Ca(f(m+1)),4);
                                Ta(MM(i),j)=lamda;                   %%%%����Ta
                                t(MM(i),j)=-(log(10*T(j,4))+P)/lamda + F(m+1);
                            end
                            if(t_path(MM(i),j)>=Z(j))
                                t(MM(i),j)=NaN;
                                fa(MM(i),j)=1;
                                for k=z:-1:1
                                    if(X(MM(i),k)==0)
                                        X(MM(i),k)=j;
                                        break;
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
        Ba=[];
        Ca=[];
        m=1;
        if(t(v,b)~=inf)
            Ba(1)=t_path(v,b);
            Ca(1)=v;
            for i=1:u
                if(isnan(t(i,b))==1&&fa(i,b)==0)
                    m=m+1;
                    Ba(m)=t_path(i,b);
                    Ca(m)=i;
                end
            end
            %         m
            if(m>1)
                [F,f]=sort(Ba);
                P=T(b,3)*F(1);
                lamda=T(b,3);
                for k=2:m
                    lamda=lamda-R(Ca(f(k-1)),4);
                    if(abs(lamda)<10^(-7))
                        lamda=0;
                    end
                    P=P+lamda*(F(k)-F(k-1));
                end
                lamda=lamda-R(Ca(f(m)),4);
                Z(b)=-(log(10*T(b,4))+P)/lamda + F(m);
                if(isnan(Z(b))==1||Z(b)<0)
                    Z(b)=inf;
                end
                for i=1:u
                    if(i==v)
                        continue;
                    else
                        if(isnan(t(i,b))==1)
                            continue;
                        end
                    end
                    if(t_path(i,b)<t(v,b))         %%��Ҫ���������´���������Ԫ�ص����ʱ��
                        Ba(m+1)=t_path(v,b);
                        Ca(m+1)=i;
                        [F,f]=sort(Ba);
                        P=T(b,3)*F(1);
                        lamda=T(b,3);
                        for k=2:(m+1)
                            lamda=lamda-R(Ca(f(k-1)),4);
                            if(abs(lamda)<10^(-7))
                                lamda=0;
                            end
                            P=P+lamda*(F(k)-F(k-1));
                        end
                        lamda=lamda-R(Ca(f(m+1)),4);
                        t(i,b)=-(log(10*T(b,4))+P)/lamda + F(m+1);
                        if(isnan(t(i,b))==1||t(i,b)<0)
                            t(i,b)=inf;
                        end
                    end
                    if(t_path(i,b)>=t(v,b)) %%�������������������
                        t(i,b)=NaN;
                        fa(i,b)=1;
                        for j=z:-1:1
                            if(X(i,j)==0)
                                X(i,j)=b;
                                break;
                            end
                        end
                    end
                end
            end
            
            if(m==1)
                Z(b)=t(v,b);
                for i=1:u
                    if(i==v)
                        continue;
                    else
                        if(isnan(t(i,b))==1)
                            continue;
                        end
                    end
                    if(t_path(i,b)<t(v,b))         %%��Ҫ���������´���������Ԫ�ص����ʱ��
                        if(t_path(i,b)<t_path(v,b))
                            %                     P=T(d,4)*exp(T(d,3)*t_path(i,d))*exp((T(d,3)-R(i,4))*(t_path(w(d),d)-t_path(i,d)));
                            %                     t_com(i,d)=log(10*P)/(R(i,4)+R(w(d),4)-T(d,3));
                            P=T(b,3)*t_path(i,b)+(T(b,3)-R(i,4))*(t_path(v,b)-t_path(i,b));
                            t_com(i,b)=(log(10*T(b,4))+P)/(R(i,4)+R(v,4)-T(b,3));
                        else
                            %                     P=T(d,4)*exp(T(d,3)*t_path(w(d),d))*exp((T(d,3)-R(w(d),4))*(t_path(i,d)-t_path(w(d),d)));
                            %                     t_com(i,d)=log(10*P)/(R(i,4)+R(w(d),4)-T(d,3));
                            P=T(b,3)*t_path(v,b)+(T(b,3)-R(v,4))*(t_path(i,b)-t_path(v,b));
                            t_com(i,b)=(log(10*T(b,4))+P)/(R(i,4)+R(v,4)-T(b,3));
                        end
                        t(i,b)=t_path(i,b)+t_com(i,b);
                        if(t_com(i,b)<=0)     %%˵������ǰ�޷������
                            t(i,b)=inf;
                        end
                    end
                    if(t_path(i,b)>=t(v,b)) %%�������������������
                        t(i,b)=NaN;
                        fa(i,b)=1;
                        for j=z:-1:1
                            if(X(i,j)==0)
                                X(i,j)=b;
                                break;
                            end
                        end
                    end
                end
            end
            
            for j=1:z
                if(j~=b)
                    if(isnan(t(v,j))==1)
                        continue;
                    end
                end
                if(j~=b)         %%���µ�ǰ������Ԫ�ص����ʱ��
                    t_path(v,j)=sqrt((T(j,1)-T(b,1))^2+(T(j,2)-T(b,2))^2)/R(v,3)+Z(b);  %%Ԥ��t_path(w(d),j)
                    Ba=[];
                    Ca=[];
                    m=0;
                    n=0;
                    for i=1:u
                        if(i==v)
                            continue;
                        end
                        if(isnan(t(i,j))==1)
                            n=n+1;
                        end
                    end
                    for i=1:u
                        if(i==v)
                            continue;
                        end
                        if(isnan(t(i,j))==1&&fa(i,j)==0)
                            m=m+1;
                            Ba(m)=t_path(i,j);
                            Ca(m)=i;
                        end
                    end
                    if(n==0)
                        if(R(v,4)>T(j,3))
                            t_com(v,j)=(log(10*T(j,4))+T(j,3)*t_path(v,j))/(R(v,4)-T(j,3));  %%�Խ���0.1Ϊ׼
                        else
                            t_com(v,j)=inf;
                        end
                        t(v,j)=t_path(v,j)+t_com(v,j);
                    end
                    if(m>0)                                        %%ֻҪ�����и����꣬���˸�������ĵ�һ�η��䣬�������и��²����������Ǻ���
                        if(t_path(v,j)<Z(j))
                            Ba(m+1)=t_path(v,j);
                            Ca(m+1)=v;
                            [F,f]=sort(Ba);
                            P=T(j,3)*F(1);
                            lamda=T(j,3);
                            for k=2:(m+1)
                                lamda=lamda-R(Ca(f(k-1)),4);
                                if(abs(lamda)<10^(-7))
                                    lamda=0;
                                end
                                P=P+lamda*(F(k)-F(k-1));
                            end
                            lamda=lamda-R(Ca(f(m+1)),4);
                            t(v,j)=-(log(10*T(j,4))+P)/lamda + F(m+1);
                            if(isnan(t(v,j))==1||t(v,j)<0)
                                t(v,j)=inf;
                            end
                        end
                        if(t_path(v,j)>=Z(j))
                            t(v,j)=NaN;
                            fa(v,j)=1;
                            for k=z:-1:1
                                if(X(v,k)==0)
                                    X(v,k)=j;
                                    break;
                                end
                            end
                        end
                    end
                end
            end
        end
        t(v,b)=nan;
        if(test==X)
            break;
        end
    end
    XX{H}=X;
end