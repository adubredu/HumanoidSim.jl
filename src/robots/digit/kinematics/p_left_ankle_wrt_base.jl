function  p_left_ankle_wrt_base_helper!(p_output1, var1)
  t3 = cos(var1[3+1])
  t6 = cos(var1[5+1])
  t9 = sin(var1[3+1])
  t7 = sin(var1[4+1])
  t10 = sin(var1[5+1])
  t14 = -1.0 * t6*t9
  t15 = t3*t7*t10
  t16 = t14 + t15
  t8 = t3*t6*t7
  t11 = t9*t10
  t12 = t8 + t11
  t4 = cos(var1[4+1])
  t18 = cos(var1[6+1])
  t22 = sin(var1[6+1])
  t24 = cos(var1[7+1])
  t25 = -1.0 * t24
  t26 = 1.0 +  t25
  t28 = sin(var1[7+1])
  t32 = t18*t16
  t33 = -1.0 * t12*t22
  t34 = t32 + t33
  t40 = t18*t12
  t41 = t16*t22
  t42 = t40 + t41
  t55 = cos(var1[8+1])
  t56 = -1.0 * t55
  t57 = 1.0 +  t56
  t59 = sin(var1[8+1])
  t74 = -1.000000637725*t26
  t75 = 1.0 +  t74
  t76 = t3*t4*t75
  t77 = -0.930418*t34*t28
  t78 = -0.366501*t42*t28
  t79 = t76 + t77 + t78
  t48 = -0.340999127418*t26*t34
  t49 = -0.134322983001*t26
  t50 = 1.0 +  t49
  t51 = t50*t42
  t52 = 0.366501*t3*t4*t28
  t53 = t48 + t51 + t52
  t63 = -0.8656776547239999*t26
  t64 = 1.0 +  t63
  t65 = t64*t34
  t66 = -0.340999127418*t26*t42
  t67 = 0.930418*t3*t4*t28
  t68 = t65 + t66 + t67
  t84 = cos(var1[9+1])
  t85 = -1.0 * t84
  t86 = 1.0 +  t85
  t88 = sin(var1[9+1])
  t91 = -1.000000637725*t57
  t92 = 1.0 +  t91
  t93 = t92*t79
  t94 = -0.930418*t53*t59
  t95 = 0.366501*t68*t59
  t96 = t93 + t94 + t95
  t102 = 0.340999127418*t57*t53
  t103 = -0.134322983001*t57
  t104 = 1.0 +  t103
  t105 = t104*t68
  t106 = -0.366501*t79*t59
  t107 = t102 + t105 + t106
  t113 = -0.8656776547239999*t57
  t114 = 1.0 +  t113
  t115 = t114*t53
  t116 = 0.340999127418*t57*t68
  t117 = 0.930418*t79*t59
  t118 = t115 + t116 + t117
  t121 = cos(var1[10+1])
  t122 = -1.0 * t121
  t123 = 1.0 +  t122
  t125 = sin(var1[10+1])
  t128 = -0.930418*t88*t96
  t129 = 0.340999127418*t86*t107
  t130 = -0.8656776547239999*t86
  t131 = 1.0 +  t130
  t132 = t131*t118
  t133 = t128 + t129 + t132
  t139 = 0.366501*t88*t96
  t140 = -0.134322983001*t86
  t141 = 1.0 +  t140
  t142 = t141*t107
  t143 = 0.340999127418*t86*t118
  t144 = t139 + t142 + t143
  t149 = -1.000000637725*t86
  t150 = 1.0 +  t149
  t151 = t150*t96
  t152 = -0.366501*t88*t107
  t153 = 0.930418*t88*t118
  t154 = t151 + t152 + t153
  t156 = cos(var1[11+1])
  t157 = -1.0 * t156
  t158 = 1.0 +  t157
  t160 = sin(var1[11+1])
  t163 = 0.930418*t125*t133
  t164 = -0.366501*t125*t144
  t165 = -1.000000637725*t123
  t166 = 1.0 +  t165
  t167 = t166*t154
  t168 = t163 + t164 + t167
  t174 = -0.8656776547239999*t123
  t175 = 1.0 +  t174
  t176 = t175*t133
  t177 = 0.340999127418*t123*t144
  t178 = -0.930418*t125*t154
  t179 = t176 + t177 + t178
  t185 = 0.340999127418*t123*t133
  t186 = -0.134322983001*t123
  t187 = 1.0 +  t186
  t188 = t187*t144
  t189 = 0.366501*t125*t154
  t190 = t185 + t188 + t189
  t193 = cos(var1[12+1])
  t194 = -1.0 * t193
  t195 = 1.0 +  t194
  t197 = sin(var1[12+1])
  t200 = 0.366501*t160*t168
  t201 = 0.340999127418*t158*t179
  t202 = -0.134322983001*t158
  t203 = 1.0 +  t202
  t204 = t203*t190
  t205 = t200 + t201 + t204
  t211 = -0.930418*t160*t168
  t212 = -0.8656776547239999*t158
  t213 = 1.0 +  t212
  t214 = t213*t179
  t215 = 0.340999127418*t158*t190
  t216 = t211 + t214 + t215
  t221 = -1.000000637725*t158
  t222 = 1.0 +  t221
  t223 = t222*t168
  t224 = 0.930418*t160*t179
  t225 = -0.366501*t160*t190
  t226 = t223 + t224 + t225
  t19 = -1.0 * t18
  t20 = 1.0 +  t19
  t255 = t3*t6
  t256 = t9*t7*t10
  t257 = t255 + t256
  t251 = t6*t9*t7
  t252 = -1.0 * t3*t10
  t253 = t251 + t252
  t27 = -0.04500040093286238*t26
  t29 = -0.0846680539949003*t28
  t30 = t27 + t29
  t35 = 1.296332362046933e-7*var1[7+1]
  t36 = 0.07877668146182712*t26
  t37 = -0.04186915633414423*t28
  t38 = t35 + t36 + t37
  t43 = -3.2909349868922137e-7*var1[7+1]
  t44 = 0.03103092645718495*t26
  t45 = -0.016492681424499736*t28
  t46 = t43 + t44 + t45
  t262 = t18*t257
  t263 = -1.0 * t253*t22
  t264 = t262 + t263
  t266 = t18*t253
  t267 = t257*t22
  t268 = t266 + t267
  t54 = 1.296332362046933e-7*var1[8+1]
  t58 = -0.14128592423750855*t57
  t60 = -0.04186915633414423*t59
  t61 = t54 + t58 + t60
  t69 = 3.2909349868922137e-7*var1[8+1]
  t70 = 0.055653945343889656*t57
  t71 = 0.016492681424499736*t59
  t72 = t69 + t70 + t71
  t80 = -0.04500040093286238*t57
  t81 = 0.15185209683981668*t59
  t82 = t80 + t81
  t87 = 0.039853038461262744*t86
  t89 = -0.23670515095269612*t88
  t90 = t87 + t89
  t280 = t4*t75*t9
  t281 = -0.930418*t264*t28
  t282 = -0.366501*t268*t28
  t283 = t280 + t281 + t282
  t270 = -0.340999127418*t26*t264
  t271 = t50*t268
  t272 = 0.366501*t4*t9*t28
  t273 = t270 + t271 + t272
  t275 = t64*t264
  t276 = -0.340999127418*t26*t268
  t277 = 0.930418*t4*t9*t28
  t278 = t275 + t276 + t277
  t98 = -1.5981976069815686e-7*var1[9+1]
  t99 = 0.08675267452931407*t86
  t100 = 0.014606169134372047*t88
  t101 = t98 + t99 + t100
  t109 = -6.295460977284962e-8*var1[9+1]
  t110 = -0.22023473313910558*t86
  t111 = -0.03707996069223323*t88
  t112 = t109 + t110 + t111
  t120 = -1.6084556086870008e-7*var1[10+1]
  t124 = -0.29135406957765553*t123
  t126 = -0.02832985722118838*t125
  t127 = t120 + t124 + t126
  t285 = t92*t283
  t286 = -0.930418*t273*t59
  t287 = 0.366501*t278*t59
  t288 = t285 + t286 + t287
  t290 = 0.340999127418*t57*t273
  t291 = t104*t278
  t292 = -0.366501*t283*t59
  t293 = t290 + t291 + t292
  t295 = t114*t273
  t296 = 0.340999127418*t57*t278
  t297 = 0.930418*t283*t59
  t298 = t295 + t296 + t297
  t135 = -4.0833068682577724e-7*var1[10+1]
  t136 = 0.11476729583292707*t123
  t137 = 0.0111594154470601*t125
  t138 = t135 + t136 + t137
  t146 = 0.03044854601678662*t123
  t147 = -0.3131431996991197*t125
  t148 = t146 + t147
  t159 = -0.26285954081199375*t158
  t161 = -0.634735404786378*t160
  t162 = t159 + t161
  t300 = -0.930418*t88*t288
  t301 = 0.340999127418*t86*t293
  t302 = t131*t298
  t303 = t300 + t301 + t302
  t305 = 0.366501*t88*t288
  t306 = t141*t293
  t307 = 0.340999127418*t86*t298
  t308 = t305 + t306 + t307
  t310 = t150*t288
  t311 = -0.366501*t88*t293
  t312 = 0.930418*t88*t298
  t313 = t310 + t311 + t312
  t170 = 6.369237629068993e-8*var1[11+1]
  t171 = -0.5905692458505322*t158
  t172 = 0.24456909227538925*t160
  t173 = t170 + t171 + t172
  t181 = 1.6169269214444473e-7*var1[11+1]
  t182 = 0.2326311605896123*t158
  t183 = -0.09633822312984319*t160
  t184 = t181 + t182 + t183
  t192 = 1.7876586242383724e-7*var1[12+1]
  t196 = 0.3243041141817093*t195
  t198 = 0.02270383571304597*t197
  t199 = t192 + t196 + t198
  t315 = 0.930418*t125*t303
  t316 = -0.366501*t125*t308
  t317 = t166*t313
  t318 = t315 + t316 + t317
  t320 = t175*t303
  t321 = 0.340999127418*t123*t308
  t322 = -0.930418*t125*t313
  t323 = t320 + t321 + t322
  t325 = 0.340999127418*t123*t303
  t326 = t187*t308
  t327 = 0.366501*t125*t313
  t328 = t325 + t326 + t327
  t207 = 7.041766963257243e-8*var1[12+1]
  t208 = -0.8232948486053725*t195
  t209 = -0.05763710717422546*t197
  t210 = t207 + t208 + t209
  t218 = 0.06194758047549556*t195
  t219 = -0.8848655643005321*t197
  t220 = t218 + t219
  t330 = 0.366501*t160*t318
  t331 = 0.340999127418*t158*t323
  t332 = t203*t328
  t333 = t330 + t331 + t332
  t335 = -0.930418*t160*t318
  t336 = t213*t323
  t337 = 0.340999127418*t158*t328
  t338 = t335 + t336 + t337
  t230 = -1.000000637725*t195
  t231 = 1.0 +  t230
  t340 = t222*t318
  t341 = 0.930418*t160*t323
  t342 = -0.366501*t160*t328
  t343 = t340 + t341 + t342
  t236 = -0.8656776547239999*t195
  t237 = 1.0 +  t236
  t242 = -0.134322983001*t195
  t243 = 1.0 +  t242
  t367 = t4*t18*t10
  t368 = -1.0 * t4*t6*t22
  t369 = t367 + t368
  t371 = t4*t6*t18
  t372 = t4*t10*t22
  t373 = t371 + t372
  t385 = -1.0 * t75*t7
  t386 = -0.930418*t369*t28
  t387 = -0.366501*t373*t28
  t388 = t385 + t386 + t387
  t380 = t64*t369
  t381 = -0.340999127418*t26*t373
  t382 = -0.930418*t7*t28
  t383 = t380 + t381 + t382
  t375 = -0.340999127418*t26*t369
  t376 = t50*t373
  t377 = -0.366501*t7*t28
  t378 = t375 + t376 + t377
  t390 = t92*t388
  t391 = 0.366501*t383*t59
  t392 = -0.930418*t378*t59
  t393 = t390 + t391 + t392
  t395 = t104*t383
  t396 = 0.340999127418*t57*t378
  t397 = -0.366501*t388*t59
  t398 = t395 + t396 + t397
  t400 = 0.340999127418*t57*t383
  t401 = t114*t378
  t402 = 0.930418*t388*t59
  t403 = t400 + t401 + t402
  t405 = -0.930418*t88*t393
  t406 = 0.340999127418*t86*t398
  t407 = t131*t403
  t408 = t405 + t406 + t407
  t410 = 0.366501*t88*t393
  t411 = t141*t398
  t412 = 0.340999127418*t86*t403
  t413 = t410 + t411 + t412
  t415 = t150*t393
  t416 = -0.366501*t88*t398
  t417 = 0.930418*t88*t403
  t418 = t415 + t416 + t417
  t420 = 0.930418*t125*t408
  t421 = -0.366501*t125*t413
  t422 = t166*t418
  t423 = t420 + t421 + t422
  t425 = t175*t408
  t426 = 0.340999127418*t123*t413
  t427 = -0.930418*t125*t418
  t428 = t425 + t426 + t427
  t430 = 0.340999127418*t123*t408
  t431 = t187*t413
  t432 = 0.366501*t125*t418
  t433 = t430 + t431 + t432
  t435 = 0.366501*t160*t423
  t436 = 0.340999127418*t158*t428
  t437 = t203*t433
  t438 = t435 + t436 + t437
  t440 = -0.930418*t160*t423
  t441 = t213*t428
  t442 = 0.340999127418*t158*t433
  t443 = t440 + t441 + t442
  t445 = t222*t423
  t446 = 0.930418*t160*t428
  t447 = -0.366501*t160*t433
  t448 = t445 + t446 + t447
  p_output1[0+1]=t101*t107 + t112*t118 - 0.2595*t12 + t127*t133 + t138*t144 + t148*t154 - 0.0002*t16 + t162*t168 + t173*t179 + t184*t190 + 0.091*t16*t20 + t199*t205 + t210*t216 + 0.091*t12*t22 + t220*t226 + 0.061947*(-0.366501*t197*t205 + 0.930418*t197*t216 + t226*t231) - 0.792446*(0.340999127418*t195*t205 - 0.930418*t197*t226 + t216*t237) + 0.402615*(0.340999127418*t195*t216 + 0.366501*t197*t226 + t205*t243) + t34*t38 - 0.0016*t3*t4 + t3*t30*t4 + t42*t46 + t53*t61 + t68*t72 + t79*t82 + t90*t96
  p_output1[1+1]=-0.2595*t253 + 0.091*t22*t253 - 0.0002*t257 + 0.091*t20*t257 + t101*t293 + t112*t298 + t127*t303 + t138*t308 + t148*t313 + t162*t318 + t173*t323 + t184*t328 + t199*t333 + t210*t338 + t220*t343 - 0.792446*(0.340999127418*t195*t333 + t237*t338 - 0.930418*t197*t343) + 0.402615*(t243*t333 + 0.340999127418*t195*t338 + 0.366501*t197*t343) + 0.061947*(-0.366501*t197*t333 + 0.930418*t197*t338 + t231*t343) + t264*t38 + t268*t46 + t273*t61 + t278*t72 + t283*t82 - 0.0016*t4*t9 + t30*t4*t9 + t288*t90
  p_output1[2+1]=t369*t38 + t101*t398 - 0.0002*t10*t4 + 0.091*t10*t20*t4 + t112*t403 + t127*t408 + t138*t413 + t148*t418 + t162*t423 + t173*t428 + t184*t433 + t199*t438 + t210*t443 + t220*t448 - 0.792446*(0.340999127418*t195*t438 + t237*t443 - 0.930418*t197*t448) + 0.402615*(t243*t438 + 0.340999127418*t195*t443 + 0.366501*t197*t448) + 0.061947*(-0.366501*t197*t438 + 0.930418*t197*t443 + t231*t448) + t373*t46 - 0.2595*t4*t6 + 0.091*t22*t4*t6 + t378*t61 + 0.0016*t7 - 1.0 * t30*t7 + t383*t72 + t388*t82 + t393*t90
end



function p_left_ankle_wrt_base(var1)
  p_output1 = zeros(3)
  p_left_ankle_wrt_base_helper!(p_output1, var1)
  return p_output1
end

