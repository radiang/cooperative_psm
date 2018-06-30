function [D,Res]=pivot(T)

for j = 1:300
A(3*j-2:3*j,1:6)=[T(1:3,1:3,j),-eye(3,3)];
B(3*j-2:3*j,1)=-[T(1,4,j), T(2,4,j), T(3,4,j)]';

end

%P_A=A'*inv(A*A'); (From Slides, Gives wrong value)
P_A=inv(A'*A)*A'; %Using pseudoinverse matrix (Different from slides,
%found on internet) 


[U,De,V]=svd(A);
De_e=De(1:6,1:6);
De(1:6,1:6)=inv(De_e);
P_A=V*De'*U';

%for i=1:300
    
%Dee=De(1:6,1:6);
%P_A=V*inv(Dee)*U'



D=P_A*B; %Use pseudoinverse to calulcate D= [btip, bpost]
Res=A*D-B; %Residual Errors

end


