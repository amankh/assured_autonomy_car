#!/usr/bin/env python
#author: qinlin@andrew.cmu.edu
import numpy as np 

formula = 'la^(-2)*(4*Isx*(l2^2*(4*Mp*(Mf+2*Mp+4*Ms)+(Mf+4*(3*Mp+Ms))*Mt+2*Mt^2)+4*Itx*(Mf+4*(Mp+Ms+Mt)))+l1^2*(l2^2*(4*Mf*Mp*(2*Mp+Ms)+8*Mp*Ms*(3*Mp+Ms)+(12*Mf*Mp+8*Mp^2+Mf*Ms+36*Mp*Ms+2*Ms^2)*Mt+2*(Mf+4*Mp+3*Ms)*Mt^2)+4*Itx*(Mf*(4*Mp+Ms+4*Mt)+2*(4*Mp^2+6*Mp*Ms+Ms^2+8*Mp*Mt+6*Ms*Mt+4*Mt^2)))+(-2)*l1^2*(2*Mp+Ms+2*Mt)*(4*Itx*(2*Mp+Ms+2*Mt)+l2^2*(Ms*Mt+2*Mp*(2*Ms+Mt)))*cos(2*x2)+(-2)*l2^2*(2*Mp+Mt)^2*(l1^2*(Mf+2*Ms)+(4*Isx+(-1)*l1^2*Ms)*cos(2*x2))*cos(2*x3)+2*l2^2*(4*Isx+(-1)*l1^2*Ms)*(2*Mp+Mt)^2*sin(2*x2)*sin(2*x3))^(-1)*(4*la*(cos(x2)*(l1*(4*Itx*(2*Mp+Ms+2*Mt)+l2^2*(4*Mp*(Mp+Ms)+(6*Mp+Ms)*Mt+Mt^2))+l2*(2*Mp+Mt)*(((-4)*Isx+l1^2*Ms)*cos(x3)+(-1)*l1*l2*(2*Mp+Mt)*cos(2*x3)))+l2*(2*Mp+Mt)*sin(x2)*((4*Isx+l1^2*(4*Mp+Ms+4*Mt))*sin(x3)+l1*l2*(2*Mp+Mt)*sin(2*x3)))*(2*Kp2*x6+g*l2*(2*Mp+Mt)*mu*sin(x1+x2+x3)+(-2)*Kp2*x3+(-2)*l2*la*Mp*sin(x2+x3)*x4^2+(-1)*l2*la*Mt*sin(x2+x3)*x4^2+(-1)*l1*l2*(2*Mp+Mt)*sin(x3)*(x4+x5)^2+(-2)*Kd2*x6)+(-1/2)*(16*Isx*(4*Itx+l2^2*(4*Mp+Mt))+4*l1^2*(4*Itx*(4*Mp+Ms+4*Mt)+l2^2*(8*Mp^2+Mt*(Ms+2*Mt)+4*Mp*(Ms+3*Mt)))+8*l1*(la*(4*Itx*(2*Mp+Ms+2*Mt)+l2^2*(4*Mp*(Mp+Ms)+(6*Mp+Ms)*Mt+Mt^2))*cos(x2)+(-1)*l2^2*(2*Mp+Mt)^2*(l1+la*cos(x2))*cos(2*x3)+l2^2*la*(2*Mp+Mt)^2*sin(x2)*sin(2*x3)))*(g*l1*(2*Mp+Ms+2*Mt)*mu*sin(x1+x2)+g*l2*(2*Mp+Mt)*mu*sin(x1+x2+x3)+2*Kp1*(x5+(-1)*x2)+(-1)*la*(l1*(2*Mp+Ms+2*Mt)*sin(x2)+l2*(2*Mp+Mt)*sin(x2+x3))*x4^2+(-2)*Kd1*x5+l1*l2*(2*Mp+Mt)*sin(x3)*x4*x6+l1*l2*(2*Mp+Mt)*sin(x3)*x5*x6+l1*l2*(2*Mp+Mt)*sin(x3)*x6*(x4+x5+x6))+2*(4*Isx*(4*Itx+l2^2*(4*Mp+Mt))+l1^2*(4*Itx*(4*Mp+Ms+4*Mt)+l2^2*(8*Mp^2+Mt*(Ms+2*Mt)+4*Mp*(Ms+3*Mt)))+(-2)*l1^2*l2^2*(2*Mp+Mt)^2*cos(2*x3))*(g*la*(Mf+2*(Mp+Ms+Mt))*sin(x(1))+g*l1*(2*Mp+Ms+2*Mt)*sin(x1+x2)+g*l2*(2*Mp+Mt)*sin(x1+x2+x3)+la*(l1*(2*Mp+Ms+2*Mt)*sin(x2)+l2*(2*Mp+Mt)*sin(x2+x3))*x4*x5+la*(l1*(2*Mp+Ms+2*Mt)*sin(x2)+l2*(2*Mp+Mt)*sin(x2+x3))*x5*(x4+x5)+l2*(2*Mp+Mt)*(l1*sin(x3)+la*sin(x2+x3))*x4*x6+l2*(2*Mp+Mt)*(l1*sin(x3)+la*sin(x(2)+x3))*x5*x6+l2*(2*Mp+Mt)*(l1*sin(x3)+la*sin(x2+x3))*x6*(x4+x5+x6)))'
index_left = []
index_right = []
for i in xrange(len(formula)):
    if formula[i]=='(':
        index_left.append(i)
    if formula[i]==')':
        index_right.append(i)
print len(index_left), len(index_right)
print formula[index_left[-1]:index_[-1]+1]

