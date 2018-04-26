function[X]=initializeX(R,T)
u=size(R,1);
z=size(T,1);
for i=1:u
    X(i,:)=randperm(z);        %%每一次随机排列编码
end