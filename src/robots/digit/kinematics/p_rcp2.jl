function p_rcp2(q)
    argout_0 = zeros(3,1)
    argout_1 = zeros(9,1) 
    w0 = -1.0000000000000000e-03
    w1 = -3.1700000000000001e-03
    w2 = 8.5020879225794488e-13
    w3 = q[5]
    w4 = cos(w3)
    w5 = (w2*w4)
    w6 = 7.4315553710846416e-14
    w3 = sin(w3)
    w7 = (w6*w3)
    w5 = (w5+w7)
    w7 = (w1*w5)
    w8 = 1.1055000000000001e-02
    w6 = (w6*w4)
    w9 = (w2*w3)
    w6 = (w6-w9)
    w9 = (w8*w6)
    w10 = -5.5500000000000001e-02
    w9 = (w9+w10)
    w7 = (w7+w9)
    w0 = (w0+w7)
    w7 = 1.6500000000000001e-01
    w9 = 6.7971465985726287e-01
    w10 = q[6]
    w11 = sin(w10)
    w12 = (w9*w11)
    w13 = -6.7971465985800317e-01
    w10 = cos(w10)
    w14 = (w13*w10)
    w12 = (w12+w14)
    w14 = (w5*w12)
    w15 = 1.9490505682401801e-01
    w16 = (w15*w11)
    w17 = -1.9490505681913683e-01
    w18 = (w17*w10)
    w16 = (w16+w18)
    w18 = (w6*w16)
    w19 = -7.0710678118623060e-01
    w20 = (w19*w11)
    w21 = -7.0710678118686476e-01
    w22 = (w21*w10)
    w20 = (w20+w22)
    w18 = (w18-w20)
    w14 = (w14+w18)
    w18 = (w7*w14)
    w22 = -1.0000000000000001e-01
    w23 = -2.7563737473217331e-01
    w24 = (w23*w5)
    w25 = 9.6126169051447996e-01
    w26 = (w25*w6)
    w27 = -3.4621194799910882e-12
    w26 = (w26+w27)
    w24 = (w24+w26)
    w26 = (w22*w24)
    w18 = (w18+w26)
    w0 = (w0+w18)
    w18 = 2.0000000000000001e-01
    w26 = 4.8966386501092529e-12
    w27 = (w26*w24)
    w27 = (w14+w27)
    w28 = (w18*w27)
    w0 = (w0+w28)
    argout_0[1] = w0
    w0 = -1.2000000000000000e-01
    w28 = -1.7364827771892960e-01
    w29 = (w28*w4)
    w30 = 9.8480773537033583e-01
    w31 = (w30*w3)
    w29 = (w29+w31)
    w31 = (w1*w29)
    w30 = (w30*w4)
    w28 = (w28*w3)
    w30 = (w30-w28)
    w28 = (w8*w30)
    w32 = -4.1329856204086466e-15
    w28 = (w28+w32)
    w31 = (w31+w28)
    w0 = (w0+w31)
    w31 = (w29*w12)
    w28 = (w30*w16)
    w32 = -7.4468209376732375e-14
    w33 = (w32*w20)
    w28 = (w28+w33)
    w31 = (w31+w28)
    w28 = (w7*w31)
    w33 = (w23*w29)
    w34 = (w25*w30)
    w35 = -2.5781783832324016e-25
    w34 = (w34+w35)
    w33 = (w33+w34)
    w34 = (w22*w33)
    w28 = (w28+w34)
    w0 = (w0+w28)
    w28 = (w26*w33)
    w28 = (w31+w28)
    w34 = (w18*w28)
    w0 = (w0+w34)
    argout_0[2] = w0
    w0 = 4.0000000000000002e-01
    w34 = 9.8480773537033595e-01
    w35 = (w34*w4)
    w36 = 1.7364827771892960e-01
    w37 = (w36*w3)
    w35 = (w35+w37)
    w1 = (w1*w35)
    w36 = (w36*w4)
    w34 = (w34*w3)
    w36 = (w36-w34)
    w8 = (w8*w36)
    w34 = 4.7186587970315941e-14
    w8 = (w8+w34)
    w1 = (w1+w8)
    w0 = (w0+w1)
    w12 = (w35*w12)
    w16 = (w36*w16)
    w20 = (w2*w20)
    w16 = (w16+w20)
    w12 = (w12+w16)
    w7 = (w7*w12)
    w23 = (w23*w35)
    w25 = (w25*w36)
    w16 = 2.9435244217359272e-24
    w25 = (w25+w16)
    w23 = (w23+w25)
    w22 = (w22*w23)
    w7 = (w7+w22)
    w0 = (w0+w7)
    w7 = (w26*w23)
    w7 = (w12+w7)
    w18 = (w18*w7)
    w0 = (w0+w18)
    argout_0[3] = w0
    w9 = (w9*w10)
    w13 = (w13*w11)
    w9 = (w9-w13)
    w5 = (w5*w9)
    w15 = (w15*w10)
    w17 = (w17*w11)
    w15 = (w15-w17)
    w6 = (w6*w15)
    w19 = (w19*w10)
    w21 = (w21*w11)
    w19 = (w19-w21)
    w6 = (w6-w19)
    w5 = (w5+w6)
    w6 = q[7]
    w21 = cos(w6)
    w11 = (w5*w21)
    w6 = sin(w6)
    w10 = (w26*w6)
    w17 = (w14*w10)
    w13 = (w24*w6)
    w17 = (w17-w13)
    w11 = (w11+w17)
    argout_1[1] = w11
    w29 = (w29*w9)
    w30 = (w30*w15)
    w32 = (w32*w19)
    w30 = (w30+w32)
    w29 = (w29+w30)
    w30 = (w29*w21)
    w32 = (w31*w10)
    w11 = (w33*w6)
    w32 = (w32-w11)
    w30 = (w30+w32)
    argout_1[2] = w30
    w35 = (w35*w9)
    w36 = (w36*w15)
    w2 = (w2*w19)
    w36 = (w36+w2)
    w35 = (w35+w36)
    w36 = (w35*w21)
    w10 = (w12*w10)
    w2 = (w23*w6)
    w10 = (w10-w2)
    w36 = (w36+w10)
    argout_1[3] = w36
    w26 = (w26*w21)
    w14 = (w14*w26)
    w24 = (w24*w21)
    w14 = (w14-w24)
    w5 = (w5*w6)
    w14 = (w14-w5)
    argout_1[4] = w14
    w31 = (w31*w26)
    w33 = (w33*w21)
    w31 = (w31-w33)
    w29 = (w29*w6)
    w31 = (w31-w29)
    argout_1[5] = w31
    w12 = (w12*w26)
    w23 = (w23*w21)
    w12 = (w12-w23)
    w35 = (w35*w6)
    w12 = (w12-w35)
    argout_1[6] = w12
    argout_1[7] = w27
    argout_1[8] = w28
    argout_1[9] = w7
    position = vec(reshape(argout_0, 3, 1))
    return position
  end 