function p_lcp3(q)
    argout_0 = zeros(3,1)
    argout_1 = zeros(9,1) 
    w0 = -1.0000000000000000e-03
    w1 = -3.1700000000000001e-03
    w2 = 8.5020879225794488e-13
    w3 = q[1]
    w4 = cos(w3)
    w5 = (w2*w4)
    w6 = -7.4315553710846416e-14
    w3 = sin(w3)
    w7 = (w6*w3)
    w5 = (w5+w7)
    w7 = (w1*w5)
    w8 = -1.1055000000000001e-02
    w6 = (w6*w4)
    w9 = (w2*w3)
    w6 = (w6-w9)
    w9 = (w8*w6)
    w10 = -5.5500000000000001e-02
    w9 = (w9+w10)
    w7 = (w7+w9)
    w0 = (w0+w7)
    w7 = -1.6500000000000001e-01
    w9 = 6.7971465985726287e-01
    w10 = q[2]
    w11 = sin(w10)
    w12 = (w9*w11)
    w13 = 6.7971465985800317e-01
    w10 = cos(w10)
    w14 = (w13*w10)
    w12 = (w12+w14)
    w14 = (w5*w12)
    w15 = -1.9490505682401801e-01
    w16 = (w15*w11)
    w17 = -1.9490505681913683e-01
    w18 = (w17*w10)
    w16 = (w16+w18)
    w18 = (w6*w16)
    w19 = -7.0710678118623060e-01
    w20 = (w19*w11)
    w21 = 7.0710678118686476e-01
    w22 = (w21*w10)
    w20 = (w20+w22)
    w18 = (w18-w20)
    w14 = (w14+w18)
    w18 = (w7*w14)
    w22 = -1.0000000000000001e-01
    w23 = -2.7563737473217331e-01
    w24 = (w23*w5)
    w25 = -9.6126169051447996e-01
    w26 = (w25*w6)
    w27 = -3.4621194799910882e-12
    w26 = (w26+w27)
    w24 = (w24+w26)
    w26 = (w22*w24)
    w18 = (w18+w26)
    w0 = (w0+w18)
    argout_0[1] = w0
    w0 = 1.2000000000000000e-01
    w18 = 1.7364827771892960e-01
    w26 = (w18*w4)
    w27 = 9.8480773537033583e-01
    w28 = (w27*w3)
    w26 = (w26+w28)
    w28 = (w1*w26)
    w27 = (w27*w4)
    w18 = (w18*w3)
    w27 = (w27-w18)
    w18 = (w8*w27)
    w29 = 4.1329856204086466e-15
    w18 = (w18+w29)
    w28 = (w28+w18)
    w0 = (w0+w28)
    w28 = (w26*w12)
    w18 = (w27*w16)
    w29 = 7.4468209376732375e-14
    w30 = (w29*w20)
    w18 = (w18+w30)
    w28 = (w28+w18)
    w18 = (w7*w28)
    w30 = (w23*w26)
    w31 = (w25*w27)
    w32 = 2.5781783832324016e-25
    w31 = (w31+w32)
    w30 = (w30+w31)
    w31 = (w22*w30)
    w18 = (w18+w31)
    w0 = (w0+w18)
    argout_0[2] = w0
    w0 = 4.0000000000000002e-01
    w18 = 9.8480773537033595e-01
    w31 = (w18*w4)
    w32 = -1.7364827771892960e-01
    w33 = (w32*w3)
    w31 = (w31+w33)
    w1 = (w1*w31)
    w32 = (w32*w4)
    w18 = (w18*w3)
    w32 = (w32-w18)
    w8 = (w8*w32)
    w18 = 4.7186587970315941e-14
    w8 = (w8+w18)
    w1 = (w1+w8)
    w0 = (w0+w1)
    w12 = (w31*w12)
    w16 = (w32*w16)
    w20 = (w2*w20)
    w16 = (w16+w20)
    w12 = (w12+w16)
    w7 = (w7*w12)
    w23 = (w23*w31)
    w25 = (w25*w32)
    w16 = 2.9435244217359272e-24
    w25 = (w25+w16)
    w23 = (w23+w25)
    w22 = (w22*w23)
    w7 = (w7+w22)
    w0 = (w0+w7)
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
    w6 = q[3]
    w21 = cos(w6)
    w11 = (w5*w21)
    w10 = 4.8966386501092529e-12
    w6 = sin(w6)
    w17 = (w10*w6)
    w13 = (w14*w17)
    w0 = (w24*w6)
    w13 = (w13+w0)
    w11 = (w11+w13)
    argout_1[1] = w11
    w26 = (w26*w9)
    w27 = (w27*w15)
    w29 = (w29*w19)
    w27 = (w27+w29)
    w26 = (w26+w27)
    w27 = (w26*w21)
    w29 = (w28*w17)
    w11 = (w30*w6)
    w29 = (w29+w11)
    w27 = (w27+w29)
    argout_1[2] = w27
    w31 = (w31*w9)
    w32 = (w32*w15)
    w2 = (w2*w19)
    w32 = (w32+w2)
    w31 = (w31+w32)
    w32 = (w31*w21)
    w17 = (w12*w17)
    w2 = (w23*w6)
    w17 = (w17+w2)
    w32 = (w32+w17)
    argout_1[3] = w32
    w32 = (w10*w21)
    w17 = (w14*w32)
    w2 = (w24*w21)
    w17 = (w17+w2)
    w5 = (w5*w6)
    w17 = (w17-w5)
    argout_1[4] = w17
    w17 = (w28*w32)
    w5 = (w30*w21)
    w17 = (w17+w5)
    w26 = (w26*w6)
    w17 = (w17-w26)
    argout_1[5] = w17
    w32 = (w12*w32)
    w21 = (w23*w21)
    w32 = (w32+w21)
    w31 = (w31*w6)
    w32 = (w32-w31)
    argout_1[6] = w32
    w24 = (w10*w24)
    w24 = (w24-w14)
    argout_1[7] = w24
    w30 = (w10*w30)
    w30 = (w30-w28)
    argout_1[8] = w30
    w10 = (w10*w23)
    w10 = (w10-w12)
    argout_1[9] = w10
    position = vec(reshape(argout_0, 3, 1))
    return position
  end 