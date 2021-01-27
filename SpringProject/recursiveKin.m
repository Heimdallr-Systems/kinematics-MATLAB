function [ITN, NNwI, dotIIrN, JN, dotJN] = recursiveKin(dotgamma,ITn, nTN, nnrN, IN_hat, IN_tilde, Jn, dotJn)
% This function is to be used for recursive calculations of analytical
% expressions to be used for manipulator kinematics
% By Nicholas Rossi
ITN = ITn*nTN;
NNwn = IN_hat*dotgamma;
dotnnrN = IN_tilde*dotgamma;
JN = [nTN.', zeros(3); -ITn*skew(nnrN), eye(3)]*Jn + [IN_hat;ITn*IN_tilde];
NNwI = JN(1:1:3,:)*dotgamma;
dotIIrN = JN(4:1:6,:)*dotgamma;
nnwI = Jn(1:1:3,:)*dotgamma;
dotJN = [-skew(NNwn)*nTN.', zeros(3); -ITn*(skew(nnwI)*skew(nnrN)+skew(dotnnrN)), zeros(3)]*Jn...
             +[nTN.', zeros(3);-ITn*skew(nnrN), eye(3)]*dotJn...
             +[zeros(3,length(dotgamma));ITn*skew(nnwI)*IN_tilde];
end