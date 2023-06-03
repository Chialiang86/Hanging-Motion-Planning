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

        INPUT0=( 
            
            # =============== hook_all_new_0 =============== # 
            # 'Hook_hcu_104_devil' 'Hook_hcu_109_devil' 'Hook_hcu_142_devil' 'Hook_hcu_149_devil' 'Hook_hcu_167_devil' 
            # 'Hook_hcu_180_devil' 'Hook_hcu_189_devil' 'Hook_hcu_279_devil' 'Hook_hcu_314_devil' 'Hook_hcu_376_devil' 
            # 'Hook_hsr_2_devil' 'Hook_hcu_54_devil' 'Hook_hcu_65_devil' 'Hook_hcu_89_devil' 'Hook_hs_109_devil' 
            # 'Hook_hs_11_devil' 'Hook_hs_134_devil' 'Hook_hs_170_devil' 'Hook_hs_174_devil' 'Hook_hs_177_devil' 
            # 'Hook_hs_191_devil' 'Hook_hs_205_devil' 'Hook_hs_232_devil' 'Hook_hs_23_devil' 'Hook_hs_260_devil'
            # 'Hook_hcu_0_normal' 'Hook_hcu_101_normal' 'Hook_hcu_104_devil' 'Hook_hcu_105_normal' 'Hook_hcu_106_normal' 
            # 'Hook_hcu_108_easy' 'Hook_hcu_109_devil' 'Hook_hcu_110_easy' 'Hook_hcu_112_hard' 'Hook_hcu_113_easy' 
            # 'Hook_hcu_118_easy' 'Hook_hcu_134_normal' 'Hook_hcu_136_hard' 'Hook_hcu_137_normal' 'Hook_hcu_138_hard' 
            # 'Hook_hcu_140_easy' 'Hook_hcu_142_devil' 'Hook_hcu_143_hard' 'Hook_hcu_147_easy' 'Hook_hcu_148_hard' 
            # 'Hook_hcu_149_devil' 'Hook_hcu_154_hard' 'Hook_hcu_158_normal' 'Hook_hcu_167_devil' 'Hook_hcu_16_easy' 
            # 'Hook_hcu_172_easy' 'Hook_hcu_176_normal' 'Hook_hcu_180_devil' 'Hook_hcu_181_hard' 'Hook_hcu_183_normal' 
            # 'Hook_hcu_189_devil' 'Hook_hcu_18_normal' 'Hook_hcu_190_easy' 'Hook_hcu_191_normal' 'Hook_hcu_198_easy' 
            # 'Hook_hcu_199_hard' 'Hook_hcu_205_easy' 'Hook_hcu_206_normal' 'Hook_hcu_207_easy' 'Hook_hcu_218_easy'
            #  'Hook_hcu_221_easy' 'Hook_hcu_227_easy' 'Hook_hcu_22_easy' 'Hook_hcu_230_normal' 'Hook_hcu_232_hard' 
            #  'Hook_hcu_237_easy' 'Hook_hcu_238_easy' 'Hook_hcu_241_normal' 'Hook_hcu_243_normal' 'Hook_hcu_245_easy' 
            #  'Hook_hcu_249_hard' 'Hook_hcu_253_normal' 'Hook_hcu_266_normal' 'Hook_hcu_267_hard' 'Hook_hcu_270_easy' 
            #  'Hook_hcu_276_hard' 'Hook_hcu_279_devil' 'Hook_hcu_286_hard' 'Hook_hcu_290_hard' 'Hook_hcu_293_easy' 
            #  'Hook_hcu_296_normal' 'Hook_hcu_300_hard' 'Hook_hcu_303_normal' 'Hook_hcu_306_normal' 'Hook_hcu_309_hard' 
            #  'Hook_hcu_313_normal' 'Hook_hcu_314_devil' 'Hook_hcu_333_hard' 'Hook_hcu_335_normal' 'Hook_hcu_338_normal' 
            #  'Hook_hcu_340_normal' 'Hook_hcu_347_easy' 'Hook_hcu_34_easy' 'Hook_hcu_351_normal' 'Hook_hcu_354_easy' 
            #  'Hook_hcu_356_hard' 'Hook_hcu_359_easy' 'Hook_hcu_363_hard' 'Hook_hcu_376_devil' 'Hook_hcu_380_hard' 
            #  'Hook_hcu_397_devil' 'Hook_hcu_399_hard' 'Hook_hcu_3_hard' 'Hook_hcu_40_hard' 'Hook_hcu_45_hard' 
            #  'Hook_hcu_54_devil' 'Hook_hcu_63_hard' 'Hook_hcu_65_devil' 'Hook_hcu_89_devil' 'Hook_hs_109_devil' 
            #  'Hook_hs_11_devil' 'Hook_hs_134_devil' 'Hook_hs_170_devil' 'Hook_hs_174_devil' 'Hook_hs_177_devil' 
            #  'Hook_hs_191_devil' 'Hook_hs_205_devil' 'Hook_hs_232_devil' 'Hook_hs_23_devil' 'Hook_hs_260_devil'
        )

        INPUT1=( 
            # =============== hook_all_new_1 =============== # 
            # 'Hook_hcu_362_easy' 'Hook_hcu_364_normal' 'Hook_hcu_386_normal' 'Hook_hcu_38_normal' 'Hook_hcu_390_easy' 
            # 'Hook_hcu_398_easy' 'Hook_hcu_46_easy' 'Hook_hcu_50_easy' 'Hook_hcu_52_easy' 'Hook_hcu_62_easy' 
            # 'Hook_hcu_69_hard' 'Hook_hcu_74_hard' 'Hook_hcu_75_easy' 'Hook_hcu_86_normal' 'Hook_hcu_8_normal' 
            # 'Hook_hs_103_normal' 'Hook_hs_105_hard' 'Hook_hs_10_easy' 'Hook_hs_110_hard' 'Hook_hs_117_hard' 
            # 'Hook_hs_120_hard' 'Hook_hs_121_easy' 'Hook_hs_124_easy' 'Hook_hs_126_easy' 'Hook_hs_137_hard' 
            # 'Hook_hs_142_normal' 'Hook_hs_148_easy' 'Hook_hs_152_easy' 'Hook_hs_154_hard' 'Hook_hs_156_hard' 
            # 'Hook_hs_157_easy' 'Hook_hs_166_hard' 'Hook_hs_176_normal' 'Hook_hs_178_normal' 'Hook_hs_180_easy' 
            # 'Hook_hs_190_easy' 'Hook_hs_193_easy' 'Hook_hs_196_hard' 'Hook_hs_198_hard' 'Hook_hs_203_easy' 
            # 'Hook_hs_206_easy' 'Hook_hs_207_hard' 'Hook_hs_208_hard' 'Hook_hs_210_normal' 'Hook_hs_216_easy' 
            # 'Hook_hs_219_easy' 'Hook_hs_21_hard' 'Hook_hs_223_easy' 'Hook_hs_229_normal' 'Hook_hs_230_hard' 
            # 'Hook_hs_233_hard' 'Hook_hs_235_normal' 'Hook_hs_236_easy' 'Hook_hs_251_normal' 'Hook_hs_252_hard' 
            # 'Hook_hs_255_normal' 'Hook_hs_25_easy' 'Hook_hs_266_devil' 'Hook_hs_275_devil' 'Hook_hs_279_normal' 
            # 'Hook_hs_286_devil' 'Hook_hs_291_hard' 'Hook_hs_293_normal' 'Hook_hs_294_devil' 'Hook_hs_304_hard' 
            # 'Hook_hs_30_devil' 'Hook_hs_310_hard' 'Hook_hs_311_normal' 'Hook_hs_321_devil' 'Hook_hs_329_hard' 
            # 'Hook_hs_331_normal' 'Hook_hs_339_devil' 'Hook_hs_342_hard' 'Hook_hs_347_devil' 'Hook_hs_348_normal' 
            # 'Hook_hs_350_normal' 'Hook_hs_354_hard' 'Hook_hs_363_devil' 'Hook_hs_364_normal' 'Hook_hs_390_normal' 
            # 'Hook_hs_393_devil' 'Hook_hs_398_devil' 'Hook_hs_43_normal' 'Hook_hs_50_normal' 'Hook_hs_75_normal' 
            # 'Hook_hs_83_normal' 'Hook_hs_92_devil' 'Hook_hs_95_devil' 'Hook_hsr_114_devil' 'Hook_hsr_115_devil' 
            # 'Hook_hsr_13_devil' 'Hook_hsr_18_devil' 'Hook_hsr_198_devil' 'Hook_hsr_218_devil' 'Hook_hsr_231_devil' 
            # 'Hook_hsr_243_devil' 'Hook_hsr_246_devil' 'Hook_hsr_258_devil' 'Hook_hsr_27_devil' 'Hook_hsr_281_devil'
        )

        INPUT2=( 
            # =============== hook_all_new_2 =============== #  
            # 'Hook_hs_277_easy' 'Hook_hs_287_easy' 'Hook_hs_28_easy' 'Hook_hs_290_easy' 'Hook_hs_314_easy' 
            # 'Hook_hs_315_easy' 'Hook_hs_317_easy' 'Hook_hs_326_easy' 'Hook_hs_327_easy' 'Hook_hs_338_easy' 
            # 'Hook_hs_349_easy' 'Hook_hs_35_easy' 'Hook_hs_361_easy' 'Hook_hs_362_easy' 'Hook_hs_370_easy' 
            # 'Hook_hs_374_easy' 'Hook_hs_382_hard' 'Hook_hs_391_hard' 'Hook_hs_396_easy' 'Hook_hs_397_hard' 
            # 'Hook_hs_42_hard' 'Hook_hs_46_hard' 'Hook_hs_48_easy' 'Hook_hs_52_hard' 'Hook_hs_54_hard' 
            # 'Hook_hs_55_hard' 'Hook_hs_58_easy' 'Hook_hs_60_easy' 'Hook_hs_70_easy' 'Hook_hs_71_hard' 
            # 'Hook_hs_74_hard' 'Hook_hs_7_hard' 'Hook_hs_80_easy' 'Hook_hs_81_easy' 'Hook_hs_86_normal' 
            # 'Hook_hs_94_hard' 'Hook_hsr_0_hard' 'Hook_hsr_101_easy' 'Hook_hsr_102_normal' 'Hook_hsr_104_hard' 
            # 'Hook_hsr_107_normal' 'Hook_hsr_109_normal' 'Hook_hsr_117_easy' 'Hook_hsr_118_hard' 'Hook_hsr_120_normal' 
            # 'Hook_hsr_123_normal' 'Hook_hsr_127_normal' 'Hook_hsr_129_normal' 'Hook_hsr_130_normal' 'Hook_hsr_137_hard' 
            # 'Hook_hsr_142_hard' 'Hook_hsr_143_hard' 'Hook_hsr_156_hard' 'Hook_hsr_15_normal' 'Hook_hsr_163_normal' 
            # 'Hook_hsr_164_hard' 'Hook_hsr_165_hard' 'Hook_hsr_166_normal' 'Hook_hsr_168_hard' 'Hook_hsr_172_hard' 
            # 'Hook_hsr_188_hard' 'Hook_hsr_191_hard' 'Hook_hsr_193_normal' 'Hook_hsr_207_normal' 'Hook_hsr_213_normal' 
            # 'Hook_hsr_217_normal' 'Hook_hsr_224_normal' 'Hook_hsr_22_normal' 'Hook_hsr_232_normal' 'Hook_hsr_242_normal' 
            # 'Hook_hsr_260_normal' 'Hook_hsr_262_normal' 'Hook_hsr_268_normal' 'Hook_hsr_275_normal' 'Hook_hsr_283_normal' 
            # 'Hook_hsr_2_devil' 'Hook_hsr_312_devil' 'Hook_hsr_318_devil' 'Hook_hsr_321_devil' 'Hook_hsr_345_devil' 
            # 'Hook_hsr_34_devil' 'Hook_hsr_371_devil' 'Hook_hsr_72_devil' 'Hook_my_180_devil' 'Hook_my_90_devil' 
            # 'Hook_omni_124_devil' 'Hook_omni_1_devil' 'Hook_omni_44_devil' 'Hook_omni_57_devil'
        )

        INPUT3=( 
            # =============== hook_all_new_3 =============== #  
            # 'Hook_hsr_11_easy' 'Hook_hsr_125_easy' 'Hook_hsr_12_easy' 'Hook_hsr_134_easy' 'Hook_hsr_135_easy' 
            # 'Hook_hsr_146_easy' 'Hook_hsr_151_easy' 'Hook_hsr_154_easy' 'Hook_hsr_170_easy' 'Hook_hsr_171_easy' 
            # 'Hook_hsr_173_easy' 'Hook_hsr_192_hard' 'Hook_hsr_194_easy' 'Hook_hsr_200_easy' 'Hook_hsr_205_hard' 
            # 'Hook_hsr_208_hard' 'Hook_hsr_211_easy' 'Hook_hsr_215_easy' 'Hook_hsr_228_hard' 'Hook_hsr_238_hard' 
            # 'Hook_hsr_241_easy' 'Hook_hsr_247_hard' 'Hook_hsr_24_easy' 'Hook_hsr_250_hard' 'Hook_hsr_254_hard' 
            # 'Hook_hsr_256_easy' 'Hook_hsr_259_hard' 'Hook_hsr_261_easy' 'Hook_hsr_263_hard' 'Hook_hsr_270_hard' 
            # 'Hook_hsr_274_easy' 'Hook_hsr_278_hard' 'Hook_hsr_282_easy' 'Hook_hsr_284_hard' 'Hook_hsr_286_hard' 
            # 'Hook_hsr_298_normal' 'Hook_hsr_300_easy' 'Hook_hsr_304_hard' 'Hook_hsr_306_easy' 'Hook_hsr_308_hard' 
            # 'Hook_hsr_313_hard' 'Hook_hsr_315_hard' 'Hook_hsr_323_easy' 'Hook_hsr_325_hard' 'Hook_hsr_330_easy' 
            # 'Hook_hsr_335_hard' 'Hook_hsr_339_normal' 'Hook_hsr_361_normal' 'Hook_hsr_363_hard' 'Hook_hsr_367_normal' 
            # 'Hook_hsr_380_hard' 'Hook_hsr_385_normal' 'Hook_hsr_391_hard' 'Hook_hsr_41_hard' 'Hook_hsr_44_normal' 
            # 'Hook_hsr_56_normal' 'Hook_hsr_5_normal' 'Hook_hsr_70_normal' 'Hook_hsr_80_hard' 'Hook_hsr_8_normal' 
            # 'Hook_hsr_97_normal' 'Hook_my_60_normal' 'Hook_omni_209_normal' 'Hook_omni_40_normal' 'Hook_omni_47_normal'
        )

        INPUT4=( 
            # =============== hook_all_new_4 =============== #  
            # 'Hook_hsr_350_easy' 'Hook_hsr_381_easy' 'Hook_hsr_390_easy' 'Hook_hsr_45_easy' 'Hook_omni_84_easy'
            # 'Hook_hsr_6_easy' 'Hook_hsr_71_easy' 'Hook_hsr_95_hard' 'Hook_hsr_98_easy' 'Hook_my_45_hard' 
            # 'Hook_my_bar_easy' 'Hook_omni_122_hard' 'Hook_omni_12_hard' 'Hook_omni_136_hard' 'Hook_omni_145_easy' 
            # 'Hook_omni_2_easy' 'Hook_omni_35_easy' 'Hook_omni_42_hard' 
        )

        INPUT5=(  
            # =============== hook_all_new_5 =============== #  
            # 'Hook_hcu_279_devil' 
            # 'Hook_hsr_27_devil' 
            # 'Hook_hsr_118_hard' 'Hook_hsr_129_normal' 'Hook_hsr_321_devil' 'Hook_omni_57_devil'
            # 'Hook_hsr_313_hard'
        )

        INFERENCE_HOOKS=(  
            # # too small
            'Hook_hky_120_devil' \
            'Hook_hky_13_hard' \
            'Hook_hky_157_devil' \
            'Hook_hky_160_devil' \
            'Hook_hky_164_devil' \
            'Hook_hky_209_devil' \
            'Hook_hky_348_devil' \
            # ====================================
            'Hook_htl_40_easy' \
            'Hook_htl_70_normal' \
            'Hook_htl_97_hard'
            'Hook_htl_100_easy' \
            'Hook_htl_105_normal' \
            'Hook_htl_106_hard' \
            'Hook_htl_133_easy' \
            'Hook_htl_147_devil' \
            'Hook_htl_152_normal' \
            # 'Hook_htl_175_devil' \ 
            'Hook_htl_175_hard' \ 
            'Hook_htl_186_hard' \
            # 'Hook_htl_194_devil' \
            'Hook_htl_194_hard' \
            'Hook_htl_209_hard' \
            'Hook_htl_222_hard' \
            'Hook_htl_232_hard' \
            'Hook_htl_284_normal' \
            'Hook_htl_337_hard' \
            # ====================================
            # 'Hook_htl_100_easy' \
            # 'Hook_htl_102_hard' \
            # 'Hook_htl_105_normal' \
            # 'Hook_htl_106_hard' \
            # 'Hook_htl_109_easy' \
            # 'Hook_htl_10_easy' \
            # 'Hook_htl_112_easy' \
            # 'Hook_htl_117_easy' \
            # 'Hook_htl_118_devil' \
            # 'Hook_htl_119_easy' \
            # 'Hook_htl_122_hard' \
            # 'Hook_htl_123_easy' \
            # 'Hook_htl_127_hard' \
            # 'Hook_htl_128_easy' \
            # 'Hook_htl_135_easy' \
            # 'Hook_htl_141_normal' \
            # 'Hook_htl_144_easy' \
            # 'Hook_htl_145_normal' \
            # 'Hook_htl_147_devil' \
            # 'Hook_htl_152_normal' \
            # 'Hook_htl_154_normal' \
            # 'Hook_htl_155_hard' \
            # 'Hook_htl_161_easy' \
            # 'Hook_htl_168_normal' \
            # 'Hook_htl_175_devil' \ 
            # 'Hook_htl_175_hard' \ 
            # 'Hook_htl_178_devil' \
            # 'Hook_htl_179_devil' \
            # 'Hook_htl_181_normal' \
            # 'Hook_htl_186_hard' \
            # 'Hook_htl_189_hard' \
            # 'Hook_htl_18_easy' \
            # 'Hook_htl_194_hard' \
            # 'Hook_htl_209_hard' \
            # 'Hook_htl_20_hard' \
            # 'Hook_htl_210_easy' \
            # 'Hook_htl_217_hard' \
            # 'Hook_htl_222_hard' \
            # 'Hook_htl_224_devil' \
            # 'Hook_htl_22_normal' \
            # 'Hook_htl_232_hard' \
            # 'Hook_htl_23_devil' \
            # 'Hook_htl_240_normal' \
            # 'Hook_htl_245_hard' \
            # 'Hook_htl_251_devil' \
            # 'Hook_htl_262_devil' \
            # 'Hook_htl_264_normal' \
            # 'Hook_htl_273_normal' \
            # 'Hook_htl_284_normal' \
            # 'Hook_htl_291_normal' \
            # 'Hook_htl_296_normal' \
            # 'Hook_htl_306_normal' \
            # 'Hook_htl_337_hard' \
            # 'Hook_htl_338_hard' \
            # 'Hook_htl_339_devil' \
            # 'Hook_htl_375_hard' \
            # 'Hook_htl_389_devil' \
            # 'Hook_htl_40_easy' \
            # 'Hook_htl_46_normal' \
            # 'Hook_htl_55_devil' \
            # 'Hook_htl_57_normal' \
            # 'Hook_htl_68_easy' \
            # 'Hook_htl_6_hard' \
            # 'Hook_htl_70_normal' \
            # 'Hook_htl_7_normal' \
            # 'Hook_htl_83_easy' \
            # 'Hook_htl_91_hard' \
            # 'Hook_htl_92_normal' \
            # 'Hook_htl_96_easy' \
            # 'Hook_htl_97_hard'
        )
        
        # python3 hanging_point_sim.py --hook-root 'models/hook_all_new' --hook Hook_my_bar_easy --output-dir everyday_objects_50

        for input in "${INPUT0[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new_0' --hook $input --output-dir 'data_all_new_0'
        done    

        for input in "${INPUT1[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new_1' --hook $input --output-dir 'data_all_new_1'
        done    

        for input in "${INPUT2[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new_2' --hook $input --output-dir 'data_all_new_2'
        done    

        for input in "${INPUT3[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new_3' --hook $input --output-dir 'data_all_new_3'
        done    

        for input in "${INPUT4[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new_4' --hook $input --output-dir 'data_all_new_4'
        done    

        for input in "${INPUT5[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/hook_all_new' --hook $input --output-dir 'data_all_new_tmp'
        done    
        for input in "${INFERENCE_HOOKS[@]}"
        do
            python3 hanging_point_sim.py --hook-root 'models/inference_hooks' --hook $input --output-dir 'inference_hooks'
        done    

    elif [ $1 = 'hangpose' ]; then 
        INPUT=(
            'data/Hook_my_bar_easy-hanging_exp/Hook_my_bar_easy-hanging_exp_bag_5.json' \
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
            # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5.json' \
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
            'data/Hook_my_bar_easy-hanging_exp/Hook_my_bar_easy-hanging_exp_mug_67.json' \
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
            'data/Hook_my_bar_easy-hanging_exp/Hook_my_bar_easy-hanging_exp_scissor_4.json' \
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
            'data/Hook_my_bar_easy-hanging_exp/Hook_my_bar_easy-hanging_exp_wrench_1.json' \
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
                
                # # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_11' \
                # # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_11' \
                # # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_11' \
                # # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_11' \
                # # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_11' \
                # # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_23' \
                # # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_23' \
                # # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_23' \
                # # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_23' \
                # # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_23' \

                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_117' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_11' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_132' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_169' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_23' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_32' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_310' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_42' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_74' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_cooking_77' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_115' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_41' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_5' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_63' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_6' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_70' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_71' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_72' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_85' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_daily_8' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_100' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_113' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_115' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_118' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_193' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_126' \
                'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_128' \
                'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_19' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_67' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_mug_90' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_8' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_19' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_27' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_31' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_39' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_48' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_58' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_74' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_79' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_scissor_95' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_10' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_127' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_12' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_1' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_27' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_31' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_32' \
                # # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_35' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_36' \
                # 'everyday_objects_50/Hook_my_bar_easy-everyday_objects_50/Hook_my_bar_easy-everyday_objects_50_tool_6'
                
            )

        # the number of init poses
        if [ $# -eq 2 ]; then 
            CNT=$2
        else 
            CNT=1000
        fi

        for input in "${INPUT[@]}"
        do
            echo "data/$input.json"
            python3 hanging_init_pose.py --input-json "$input.json" --max-cnt $CNT
        done
    
    elif [ $1 = 'kptraj' ]; then 

        for i in {11..20}
        do 
           
            python3 keypoint_trajectory.py -dr data_all -kd kptraj_all-1 > "kptraj_logs/kptraj_all-${i}.txt"
        
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
            # 'data/data_all_new_testing/Hook_hcu_190_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_293_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_359_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_362_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_390_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_75_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_190_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_216_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_314_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_317_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_370_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_70_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_125_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_381_easy-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_71_easy-everyday_objects'

            # 'data/data_all_new_testing/Hook_hcu_134_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_243_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_296_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_303_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_306_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_335_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_364_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_229_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_293_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_123_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_15_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_22_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_298_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_56_normal-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_5_normal-everyday_objects'

            'data/data_all_new_testing/Hook_hcu_138_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hcu_181_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hcu_380_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hcu_3_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_105_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_117_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_154_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_156_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_42_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hs_94_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hsr_118_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hsr_263_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hsr_304_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hsr_335_hard-everyday_objects'
            'data/data_all_new_testing/Hook_hsr_391_hard-everyday_objects'

            # 'data/data_all_new_testing/Hook_hcu_104_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_279_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_376_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hcu_89_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_275_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_339_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_363_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_393_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hs_95_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_13_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_218_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_312_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_321_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_hsr_371_devil-everyday_objects'
            # 'data/data_all_new_testing/Hook_omni_124_devil-everyday_objects'
        )

        length=${#INPUT[@]}
        
        # for input in "${INPUT[@]}"
        # do

        for (( i=0; i<$length; i++ )) 
        do
            LOG_FILE="nce_package/res_hard_${i}_new.txt"
            python3 hanging_env_hce.py -ir ${INPUT[$i]} > $LOG_FILE
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
            'everyday_objects_50_cooking_11.json'
            'everyday_objects_50_cooking_117.json'
            'everyday_objects_50_cooking_132.json'
            'everyday_objects_50_cooking_169.json'
            'everyday_objects_50_cooking_23.json'
            'everyday_objects_50_cooking_310.json'
            'everyday_objects_50_cooking_32.json'
            'everyday_objects_50_cooking_42.json'
            'everyday_objects_50_cooking_74.json'
            'everyday_objects_50_cooking_77.json'
            'everyday_objects_50_daily_115.json'
            'everyday_objects_50_daily_41.json'
            'everyday_objects_50_daily_5.json'
            'everyday_objects_50_daily_6.json'
            'everyday_objects_50_daily_63.json'
            'everyday_objects_50_daily_70.json'
            'everyday_objects_50_daily_71.json'
            'everyday_objects_50_daily_72.json'
            'everyday_objects_50_daily_8.json'
            'everyday_objects_50_daily_85.json'
            'everyday_objects_50_mug_100.json'
            'everyday_objects_50_mug_113.json'
            'everyday_objects_50_mug_115.json'
            'everyday_objects_50_mug_118.json'
            'everyday_objects_50_mug_126.json'
            'everyday_objects_50_mug_128.json'
            'everyday_objects_50_mug_19.json'
            'everyday_objects_50_mug_193.json'
            'everyday_objects_50_mug_67.json'
            'everyday_objects_50_mug_90.json'
            'everyday_objects_50_scissor_19.json'
            'everyday_objects_50_scissor_27.json'
            'everyday_objects_50_scissor_31.json'
            'everyday_objects_50_scissor_39.json'
            'everyday_objects_50_scissor_48.json'
            'everyday_objects_50_scissor_58.json'
            'everyday_objects_50_scissor_74.json'
            'everyday_objects_50_scissor_79.json'
            'everyday_objects_50_scissor_8.json'
            'everyday_objects_50_scissor_95.json'
            'everyday_objects_50_tool_1.json'
            'everyday_objects_50_tool_10.json'
            'everyday_objects_50_tool_12.json'
            'everyday_objects_50_tool_127.json'
            'everyday_objects_50_tool_27.json'
            'everyday_objects_50_tool_31.json'
            'everyday_objects_50_tool_32.json'
            'everyday_objects_50_tool_35.json'
            'everyday_objects_50_tool_36.json'
            'everyday_objects_50_tool_6.json'
        )
        # HOOK=($(ls keypoint_trajectory/everyday_objects_50/ | grep Hook))
        # HOOK=(
        #     'Hook_my_bar_easy.json'
        # )
        HOOK=(
            'Hook_hcu_104_devil.json' 'Hook_hcu_134_normal.json' 'Hook_hcu_138_hard.json' 'Hook_hcu_181_hard.json' 'Hook_hcu_190_easy.json' 
            'Hook_hcu_243_normal.json' 'Hook_hcu_279_devil.json' 'Hook_hcu_293_easy.json' 'Hook_hcu_296_normal.json' 'Hook_hcu_303_normal.json' 
            'Hook_hcu_306_normal.json' 'Hook_hcu_335_normal.json' 'Hook_hcu_359_easy.json' 'Hook_hcu_362_easy.json' 'Hook_hcu_364_normal.json' 
            'Hook_hcu_376_devil.json' 'Hook_hcu_380_hard.json' 'Hook_hcu_390_easy.json' 'Hook_hcu_3_hard.json' 'Hook_hcu_75_easy.json' 
            'Hook_hcu_89_devil.json' 'Hook_hs_105_hard.json' 'Hook_hs_117_hard.json' 'Hook_hs_154_hard.json' 'Hook_hs_156_hard.json' 
            'Hook_hs_190_easy.json' 'Hook_hs_216_easy.json' 'Hook_hs_229_normal.json' 'Hook_hs_275_devil.json' 'Hook_hs_293_normal.json' 
            'Hook_hs_314_easy.json' 'Hook_hs_317_easy.json' 'Hook_hs_339_devil.json' 'Hook_hs_363_devil.json' 'Hook_hs_370_easy.json' 
            'Hook_hs_393_devil.json' 'Hook_hs_42_hard.json' 'Hook_hs_70_easy.json' 'Hook_hs_94_hard.json' 'Hook_hs_95_devil.json' 
            'Hook_hsr_118_hard.json' 'Hook_hsr_123_normal.json' 'Hook_hsr_125_easy.json' 'Hook_hsr_13_devil.json' 'Hook_hsr_15_normal.json' 
            'Hook_hsr_218_devil.json' 'Hook_hsr_22_normal.json' 'Hook_hsr_263_hard.json' 'Hook_hsr_298_normal.json' 'Hook_hsr_304_hard.json' 
            'Hook_hsr_312_devil.json' 'Hook_hsr_321_devil.json' 'Hook_hsr_335_hard.json' 'Hook_hsr_371_devil.json' 'Hook_hsr_381_easy.json' 
            'Hook_hsr_391_hard.json' 'Hook_hsr_56_normal.json' 'Hook_hsr_5_normal.json' 'Hook_hsr_71_easy.json' 'Hook_omni_124_devil.json'
        )

        # rm "keypoint_trajectory/result.txt"
        for hook in "${HOOK[@]}"
        do
            for object in "${OBJECT[@]}"
            do
                echo "pair : $object $hook"
                python3 hanging_by_trajectory.py -dd 'everyday_objects_50' -id 'everyday_objects_50' --traj_id 10 --wpt_num 40 --wpt_dim 6 -obj $object -hook $hook # -sg
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
