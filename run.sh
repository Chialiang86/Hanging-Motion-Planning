# bin/sh

if [ $# -ge 1 ]; then

    if [ $1 = 'stl2obj' ]; then 
        
        INPUT=('models/hook/Hook_60/Hook_60.stl' 'models/hook/Hook_90/Hook_90.stl' 'models/hook/Hook_180/Hook_180.stl' 'models/hook/Hook_bar/Hook_bar.stl' 'models/hook/Hook_long/Hook_long.stl' 'models/hook/Hook_skew/Hook_skew.stl')
        RAW=('models/hook/Hook_60/Hook_60.obj' 'models/hook/Hook_90/Hook_90.obj' 'models/hook/Hook_180/Hook_180.obj' 'models/hook/Hook_bar/Hook_bar.obj' 'models/hook/Hook_long/Hook_long.obj' 'models/hook/Hook_skew/Hook_skew.obj')
        BLENDER='/home/chialiang/Downloads/blender-2.93.2-linux-x64/blender'
        VHACD='/home/chialiang/v-hacd/app/TestVHACD'
        for input in "${INPUT[@]}"
        do
            $BLENDER -noaudio --background --python utils/stl2obj.py -- $input
        done
        # for raw in "${RAW[@]}"
        # do
        #     $VHACD $raw -o obj
        # done

    elif [ $1 = 'vhacd' ]; then 
        
        INPUT=('models/geo_data/hanging_exp')
        # INPUT=('models/hook')
        for input in "${INPUT[@]}"
        do
            python3 utils/vhacd.py --input-dir $input
        done

    elif [ $1 = 'obj2urdf' ]; then 
        
        # INPUT=('models/geo_data/hanging_exp')
        INPUT=('models/hook')
        for input in "${INPUT[@]}"
        do
            python3 utils/obj2urdf.py --input-dir $input
        done

    elif [ $1 = 'hangsim' ]; then 
        
        INPUT=(
            # 'Hook1' 'Hook_60' 'Hook_hcu_127' 'Hook_hcu_176' 'Hook_hcu_230' 
            # 'Hook_hcu_286' 'Hook_hcu_344' 'Hook_hcu_398' 'Hook_hcu_85' 'Hook12' 'Hook84' 'Hook_hcu_132' 
            # 'Hook_hcu_18' 'Hook_hcu_232' 
            'Hook_hcu_290' 
            # 'Hook_hcu_347' 'Hook_hcu_399' 'Hook_hcu_86' 
            # 'Hook122' 'Hook_90' 'Hook_hcu_134' 
            'Hook_hcu_180' 
            # 'Hook_hcu_236' 'Hook_hcu_293' 'Hook_hcu_351' 'Hook_hcu_40' 
            'Hook_hcu_89' 
            # 'Hook124' 'Hook_bar' 
            # 'Hook_hcu_136' 'Hook_hcu_181' 'Hook_hcu_237' 'Hook_hcu_296' 'Hook_hcu_354' 'Hook_hcu_45' 'Hook_hcu_98' 
            # 'Hook136' 'Hook_hcu_0' 'Hook_hcu_137' 'Hook_hcu_183' 'Hook_hcu_238' 'Hook_hcu_3' 'Hook_hcu_356' 'Hook_hcu_46' 
            # 'Hook145' 'Hook_hcu_1' 'Hook_hcu_138' 
            'Hook_hcu_189' 
            # 'Hook_hcu_241' 'Hook_hcu_300' 'Hook_hcu_359' 'Hook_hcu_50' 
            # 'Hook15' 'Hook_hcu_100' 'Hook_hcu_140' 'Hook_hcu_190' 'Hook_hcu_243' 'Hook_hcu_303' 'Hook_hcu_362' 'Hook_hcu_52' 
            # 'Hook_180' 'Hook_hcu_101' 'Hook_hcu_141' 'Hook_hcu_191' 'Hook_hcu_245' 'Hook_hcu_306' 'Hook_hcu_363' 'Hook_hcu_54' 
            # 'Hook186' 'Hook_hcu_104' 
            'Hook_hcu_142' 
            # 'Hook_hcu_198' 'Hook_hcu_249' 'Hook_hcu_309' 'Hook_hcu_364' 'Hook_hcu_55' 
            # 'Hook2' 'Hook_hcu_105' 'Hook_hcu_143' 'Hook_hcu_199' 'Hook_hcu_251' 'Hook_hcu_313' 'Hook_hcu_376' 'Hook_hcu_62' 
            # 'Hook209' 'Hook_hcu_106' 'Hook_hcu_147' 'Hook_hcu_205' 'Hook_hcu_253' 'Hook_hcu_314' 'Hook_hcu_38' 'Hook_hcu_63' 
            # 'Hook23' 'Hook_hcu_107' 'Hook_hcu_148' 'Hook_hcu_206' 'Hook_hcu_266' 'Hook_hcu_323' 'Hook_hcu_380' 'Hook_hcu_65' 
            # 'Hook35' 'Hook_hcu_108' 'Hook_hcu_149' 'Hook_hcu_207' 'Hook_hcu_267' 'Hook_hcu_328' 'Hook_hcu_381' 'Hook_hcu_69' 
            # 'Hook40' 
            'Hook_hcu_109' 
            # 'Hook_hcu_154' 'Hook_hcu_218' 'Hook_hcu_270' 
            'Hook_hcu_333' 
            'Hook_hcu_383' 
            # 'Hook_hcu_7' 'Hook42' 'Hook_hcu_110' 'Hook_hcu_158' 'Hook_hcu_219' 'Hook_hcu_271' 
            # 'Hook_hcu_335' 'Hook_hcu_386' 'Hook_hcu_74' 
            # 'Hook44' 'Hook_hcu_112' 'Hook_hcu_16' 'Hook_hcu_22' 'Hook_hcu_276' 'Hook_hcu_338' 'Hook_hcu_390' 'Hook_hcu_75' 
            # 'Hook47' 'Hook_hcu_113' 'Hook_hcu_167' 'Hook_hcu_221' 'Hook_hcu_277' 'Hook_hcu_34' 'Hook_hcu_391' 'Hook_hcu_76' 
            # 'Hook57' 'Hook_hcu_118' 'Hook_hcu_172' 'Hook_hcu_227' 'Hook_hcu_279' 'Hook_hcu_340' 'Hook_hcu_397'
            # 'Hook_hcu_8' 'Hook_hs_137' 'Hook_hs_198' 'Hook_hs_236' 'Hook_hs_293' 
            'Hook_hs_347' 
            # 'Hook_hs_396' 'Hook_hs_80'
            # 'Hook_hs_142' 'Hook_hs_203' 'Hook_hs_25' 'Hook_hs_294' 'Hook_hs_348' 'Hook_hs_397' 'Hook_hs_81'
            # 'Hook_hs_148' 
            'Hook_hs_205' 
            # 'Hook_hs_250' 'Hook_hs_30' 'Hook_hs_349' 'Hook_hs_398' 'Hook_hs_83'
            # 'Hook_hs_152' 'Hook_hs_206' 'Hook_hs_251' 'Hook_hs_304' 'Hook_hs_35' 'Hook_hs_42' 'Hook_hs_86'
            # 'Hook_hs_10' 'Hook_hs_154' 'Hook_hs_207' 'Hook_hs_252' 'Hook_hs_310' 'Hook_hs_350' 'Hook_hs_43' 
            'Hook_hs_92'
            # 'Hook_hs_103' 'Hook_hs_156' 'Hook_hs_208' 'Hook_hs_255' 'Hook_hs_311' 'Hook_hs_354' 'Hook_hs_46' 'Hook_hs_94'
            # 'Hook_hs_105' 'Hook_hs_157' 
            'Hook_hs_209' 
            'Hook_hs_260' 
            # 'Hook_hs_314' 'Hook_hs_361' 'Hook_hs_48' 'Hook_hs_95'
            # 'Hook_hs_109' 'Hook_hs_166' 'Hook_hs_21' 'Hook_hs_263' 'Hook_hs_315' 'Hook_hs_362' 'Hook_hs_50' 'Hook_skew'
            # 'Hook_hs_11' 'Hook_hs_170' 'Hook_hs_210' 'Hook_hs_266' 'Hook_hs_317' 'Hook_hs_363' 'Hook_hs_52'
            # 'Hook_hs_110' 'Hook_hs_174' 'Hook_hs_216' 'Hook_hs_275' 'Hook_hs_321' 'Hook_hs_364' 'Hook_hs_54'
            # 'Hook_hs_113' 'Hook_hs_176' 'Hook_hs_219' 'Hook_hs_277' 'Hook_hs_326' 'Hook_hs_367' 'Hook_hs_55'
            # 'Hook_hs_117' 'Hook_hs_177' 'Hook_hs_223' 'Hook_hs_279' 'Hook_hs_327' 'Hook_hs_370' 'Hook_hs_58'
            # 'Hook_hs_119' 'Hook_hs_178' 'Hook_hs_229' 'Hook_hs_28' 'Hook_hs_328' 'Hook_hs_374' 'Hook_hs_60'
            'Hook_hs_120' 
            # 'Hook_hs_180' 'Hook_hs_23' 'Hook_hs_286' 'Hook_hs_329' 'Hook_hs_382' 'Hook_hs_7'
            # 'Hook_hs_121' 'Hook_hs_190' 'Hook_hs_230' 'Hook_hs_287' 'Hook_hs_331' 'Hook_hs_390' 'Hook_hs_70'
            # 'Hook_hs_124' 
            'Hook_hs_191' 
            #'Hook_hs_232' 'Hook_hs_290' 'Hook_hs_338' 'Hook_hs_391' 'Hook_hs_71'
            # 'Hook_hs_126' 'Hook_hs_193' 'Hook_hs_233' 'Hook_hs_291' 'Hook_hs_339' 'Hook_hs_392' 'Hook_hs_74'
            # 'Hook_hs_134' 'Hook_hs_196' 'Hook_hs_235' 'Hook_hs_292' 'Hook_hs_342' 'Hook_hs_393' 'Hook_hs_75'
        )
        for input in "${INPUT[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_hard' --hook $input --output-dir 'data_hard'
        done    

    elif [ $1 = 'hangpose' ]; then 
        INPUT=(
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_6.json' \
            #1.2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_70.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_106.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_114.json' \
            #0.8
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_115.json' \
            # ?'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_118.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_2.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_41.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_42.json' \
            'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5.json' \
            #1.0
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_63.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_71.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_72.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_7.json' \
            #0.8
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_84.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_85.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_8.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_97.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_100.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_112.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_113.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_115.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_118.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_11.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_123.json' \
            #1.2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_126.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_128.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_129.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_132.json' \
            #1.2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_135.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_142.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_145.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_146.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_147.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_149.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_150.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_159.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_166.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_173.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_181.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_184.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_193.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_199.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_19.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_204.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_205.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_43.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_64.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_67.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_70.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_73.json' \
            #1.2 
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_80.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_82.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_90.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_101.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_12.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_14.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_19.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_22.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_27.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_31.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_39.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_48.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_58.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_62.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_74.json' \
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_79.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_8.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_92.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_95.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_98.json' \
            #1.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_10.json' \
            #3
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_12.json' \
            # ?'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_16.json' \
            #3
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_17.json' \
            #3
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1.json' \
            #3
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_25.json' \
            #2.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_27.json' \
            #3
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_31.json' \
            #2.5
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_32.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_35.json' \
            #2
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_36.json'
        )

        for input in "${INPUT[@]}"
        do
            python3 hanging_pose_render.py --input-json $input
        done

    elif [ $1 = 'initpose' ]; then 
        
        INPUT=( 
                
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_11' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_11' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_11' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_11' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_23' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_23' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_23' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_23' \

                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_6" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_70" , \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_106" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_114" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_115" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_2" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_41" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_42" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_63" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_71" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_72" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_7" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_84" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_85" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_8" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_97" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_100" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_112" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_113" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_115" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_118" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_11" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_123" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_126" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_128" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_129" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_132" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_135" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_142" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_145" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_146" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_147" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_149" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_150" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_159" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_166" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_173" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_181" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_184" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_193" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_199" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_19" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_204" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_205" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_43" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_64" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_67" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_70" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_73" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_80" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_82" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_90" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_101" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_12" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_14" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_19" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_22" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_27" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_31" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_39" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_48" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_58" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_62" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_74" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_79" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_8" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_92" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_95" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_98" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_10" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_12" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_17" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_25" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_27" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_31" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_32" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_35" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_36" \

                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5" \
                # "data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_5" \
                # "data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_5" \
                # "data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5" \
                # "data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5" \
                # "data/Hook1-hanging_exp/Hook1-hanging_exp_daily_5" \
                # "data/Hook2-hanging_exp/Hook2-hanging_exp_daily_5" \
                # ddd"data/Hook12-hanging_exp/Hook12-hanging_exp_daily_5" \
                # "data/Hook15-hanging_exp/Hook15-hanging_exp_daily_5" \
                # "data/Hook22-hanging_exp/Hook22-hanging_exp_daily_5" \
                # "data/Hook23-hanging_exp/Hook23-hanging_exp_daily_5" \
                # "data/Hook35-hanging_exp/Hook35-hanging_exp_daily_5" \
                # "data/Hook40-hanging_exp/Hook40-hanging_exp_daily_5" \
                # "data/Hook42-hanging_exp/Hook42-hanging_exp_daily_5" \
                # "data/Hook44-hanging_exp/Hook44-hanging_exp_daily_5" \
                # "data/Hook47-hanging_exp/Hook47-hanging_exp_daily_5" \
                # "data/Hook48-hanging_exp/Hook48-hanging_exp_daily_5" \
                # "data/Hook57-hanging_exp/Hook57-hanging_exp_daily_5" \
                # "data/Hook67-hanging_exp/Hook67-hanging_exp_daily_5" \
                # "data/Hook84-hanging_exp/Hook84-hanging_exp_daily_5" \
                # "data/Hook122-hanging_exp/Hook122-hanging_exp_daily_5" \
                # "data/Hook124-hanging_exp/Hook124-hanging_exp_daily_5" \
                # "data/Hook136-hanging_exp/Hook136-hanging_exp_daily_5" \
                # "data/Hook145-hanging_exp/Hook145-hanging_exp_daily_5" \
                # "data/Hook186-hanging_exp/Hook186-hanging_exp_daily_5" \
                # "data/Hook209-hanging_exp/Hook209-hanging_exp_daily_5" \
                
                
                "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_19" \
            )

        # the number of init poses
        if [ $# -eq 2 ]; then 
            CNT=$2
        else 
            CNT=1000
        fi

        for input in "${INPUT[@]}"
        do
            python3 hanging_init_pose.py --input-json "data/$input.json" --max-cnt $CNT
        done
    
    elif [ $1 = 'render' ]; then 
        
        INPUT=( 
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5' \
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_5' \
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_mug_59' \
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_scissor_4' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_bag_5' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_5' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_mug_59' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_scissor_4' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_bag_5' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_mug_59' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_scissor_4' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_bag_5' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_mug_59' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_scissor_4' \
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_wrench_1' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_wrench_1' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_wrench_1' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_wrench_1'
            )
        for input in "${INPUT[@]}"
        do
            python3 hanging_pose_render.py --input-json "$input.json"
        done
    
    elif [ $1 = 'hangenv' ]; then 
        
        max=1
        # INPUT='data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5'
        INPUT=( 
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_5' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_mug_59' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_scissor_4' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_bag_5' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_5' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_mug_59' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_scissor_4' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_bag_5' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_mug_59' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_scissor_4' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_bag_5' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_mug_59' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_scissor_4' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_wrench_1' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_wrench_1' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_wrench_1'
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_wrench_1' \

                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_6" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_70" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_106" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_114" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_115" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_2" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_41" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_42" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_63" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_71" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_72" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_7" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_84" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_85" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_8" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_97" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_100" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_112" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_113" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_115" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_118" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_11" , \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_123" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_126" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_128" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_129" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_132" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_135" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_142" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_145" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_146" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_147" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_149" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_150" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_159" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_166" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_173" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_181" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_184" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_193" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_199" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_19" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_204" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_205" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_43" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_64" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_67" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_70" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_73" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_80" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_82" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_90" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_101" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_12" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_14" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_19" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_22" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_27" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_31" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_39" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_48" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_58" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_62" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_74" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_79" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_8" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_92" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_95" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_98" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_10" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_12" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_17" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_25" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_27" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_31" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_32" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_35" \
                # "data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_36" \
            )
        for input in "${INPUT[@]}"
        do
            # rm "${input}_easy.csv"
            # rm "${input}_medium.csv"
            # rm "${input}_hard.csv"
            for i in `seq 1 $max`
            do
                python3 hanging_env.py --input-json "$input.json" --id $i > "$input.txt"
                python3 extract_log.py --input "$input.txt" --output "$input" 
                # rm "$input.txt"
            done
        done
    
    elif [ $1 = 'kptpose' ]; then 

        max=1
        for i in `seq 1 $max`
        do
            echo "$i"
            python3 keypoint_pose.py
        done
   
    elif [ $1 = 'hangtraj' ]; then 

        OBJECT=(
            # 'hanging_exp_bag_5.json' \
            'hanging_exp_daily_5.json' \
            # 'hanging_exp_mug_59.json' \
            # 'hanging_exp_scissor_4.json' \
            # 'hanging_exp_wrench_1.json'
        )
        HOOK=($(ls keypoint_trajectory/keypoint_trajectory_1104/Hook_90* | sed 's|.*/||'))

        rm "keypoint_trajectory/result.txt"
        for hook in "${HOOK[@]}"
        do
            for object in "${OBJECT[@]}"
            do
                echo "pair : $object $hook"
                python3 hanging_by_trajectory.py -dd data -id keypoint_trajectory_1104 -sg -od 1104 -obj $object -hook $hook
            done
        done
    
    elif [ $1 = 'showobj' ]; then 

        OBJ_DIRS=(
            'models/hook' \
            # 'models/geo_data/hanging_exp' \
            # 'models/geo_data/wrench' \
            # 'models/geo_data/scissor' \
            # 'models/geo_data/mug' \
            # 'models/geo_data/daily_object' \
        )
        for OBJ_DIR in "${OBJ_DIRS[@]}"
        do
            python3 visualization/show_model.py --input-dir $OBJ_DIR
        done
    
    elif [ $1 = 'showpcd' ]; then 

        OBJ_DIRS=(
            'models/test' \
            # 'models/hook' \
            # 'models/geo_data/hanging_exp' \
            # 'models/geo_data/wrench' \
            # 'models/geo_data/scissor' \
            # 'models/geo_data/mug' \
            # 'models/geo_data/daily_object' \
        )
        for OBJ_DIR in "${OBJ_DIRS[@]}"
        do
            python3 visualization/show_pcd.py --input-dir $OBJ_DIR
        done
    
    elif [ $1 = 'grep' ]; then 

        OBJS=(
            'bag' 'mug' 'daily' 'scissor' 'wrench'
        )
        HOOKS=(
            '60' 'bar' 'skew' '90' '180'
        )
        for OBJ in "${OBJS[@]}"
        do
            for HOOK in "${HOOKS[@]}"
            do
                echo $OBJ $HOOK
                ls demonstration_data/ | grep ".*$HOOK.*$OBJ.*" | wc -l
            done
        done
    else 
        echo "nope"
    fi

else 
    echo "should input at least one argument"
fi
