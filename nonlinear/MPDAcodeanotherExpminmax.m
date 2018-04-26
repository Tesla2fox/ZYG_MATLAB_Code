%u=3;z=3;  %%u=rob_number,z=task_number
function[XX]=MPDAcodeanotherExpminmax(R,T)
format short g
q=11;
for H=1:q
    u=size(R,1);
    z=size(T,1);
    Q=u*z;
    t=zeros(u,z);      %t=[t11 t12 t13;t21 t22 t23];
    E=zeros(u,z);
    A=[];
    B=[];
    C=[];
    D=[];
    t_path=zeros(u,z);%%[tp11 tp12 tp13;tp21 tp22 tp23];         %%除11和21，均为从上一个任务点去下一个任务点，即机器人的位置在更新
    t_com=zeros(u,z);%%[tc11 tc12 tc13;tc21 tc22 tc23];
    X=zeros(u,z);
    omega=(H-1)/(q-1);     %%ω
    
    
    
    for i=1:u
        for j=1:z
            t_path(i,j)=sqrt((T(j,1)-R(i,1))^2+(T(j,2)-R(i,2))^2)/R(i,3);
            if(R(i,4)<=T(j,3))
                t_com(i,j)=inf;
            else
                t_com(i,j)=(log(10*T(j,4))+T(j,3)*t_path(i,j))/(R(i,4)-T(j,3));  %%以降到0.1为准
            end
            t(i,j)=t_path(i,j)+t_com(i,j);
        end
    end
    p=1;
    for i=1:u
        for j=1:z
            A(p)=t_path(i,j);
            B(p)=t(i,j);
            p=p+1;
        end
    end
    [e,x]=sort(A);
    [g,s]=sort(B,'descend');        %%完成时间为有限值，越小越好   %%%%
    r=1;
    for i=1:Q
        C(i)=r;
        if(i==Q)
            break;
        end
        if(e(i+1)~=e(i))
            r=r+1;
        end
    end
    if(g(Q)==inf)          %%所有任务单个机器人都不能完成  C和D是最后需要计算的矩阵
        B=[];
        p=1;
        for i=1:u
            for j=1:z
                E(i,j)=R(i,4)+T(j,3);
                B(p)=E(i,j);
                p=p+1;
            end
        end
        [h,s]=sort(B,'descend');
        r=1;
        for i=1:Q
            D(i)=r;
            if(i==Q)
                break;
            else
                if(h(i+1)~=h(i))
                    r=r+1;
                end
            end
        end
    end
    
    if(g(1)~=inf)
        r=1;
        for i=1:Q
            D(i)=r;
            if(i==Q)
                break;
            else
                if(g(i+1)~=g(i))
                    r=r+1;
                end
            end
        end
    end
    
    if(g(Q)~=inf&&g(1)==inf)
        for i=2:Q
            if(g(i)~=inf)
                f=i;           %%f是第一个不能完成的R-T对
                break;
            end
        end
        F=[];
        n=1;
        for i=1:f-1
            v=ceil(s(i)/z);       %%对应的机器人编号
            b=rem(s(i),z);        %%对应的任务点编号
            if(b==0)
                b=z;
            end
            F(n)=R(v,4)+T(b,3);
            n=n+1;
        end
        [w,l]=sort(F,'descend');
        n=1;
        ss=s;
        for i=1:f-1
            s(i)=ss(l(n));
            n=n+1;
        end
        r=1;
        for i=1:f-1
            D(i)=r;
            if(i==f-1)
                break;
            else
                if(w(l(i+1))~=w(l(i)))
                    r=r+1;
                end
            end
        end
        r=r+1;
        for i=f:Q
            D(i)=r;
            if(i==Q)
                break;
            else
                if(g(i+1)~=g(i))
                    r=r+1;
                end
            end
        end
    end
    P=zeros(1,Q);
    for i=1:Q
        for j=1:Q
            if(x(i)==s(j))
                P(x(i))=omega*C(i)+(1-omega)*D(j);
                break;
            end
        end
    end
    [~,hh]=sort(P);
    for i=1:Q
        v=ceil(hh(i)/z);       %%对应的机器人编号
        b=rem(hh(i),z);        %%对应的任务点编号
        if(b==0)
            b=z;
        end
        for k=1:z
            if(X(v,k)==0)
                X(v,k)=b;
                break;
            end
        end
    end
    XX{H}=X;
end
