function[X]=initializeX(R,T)
u=size(R,1);
z=size(T,1);
for i=1:u
    X(i,:)=randperm(z);        %%ÿһ��������б���
end