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
            python3 vhacd.py --input-dir $input
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
            # 'Hook1' 
            # 'Hook12' 
            # 'Hook122' 
            # 'Hook124' 
            # 'Hook136' 
            # 'Hook145' 
            # 'Hook15' 
            # 'Hook186' 
            # 'Hook2' 
            # 'Hook209' 
            # 'Hook23' 
            # 'Hook35' 
            # 'Hook40' 
            # 'Hook42' 
            # 'Hook44' 
            # 'Hook47' 
            # 'Hook57' 
            # 'Hook84' 
            # 'Hook_180' 'Hook_60' 'Hook_90' 'Hook_bar' 'Hook_skew' 
        )
        for input in "${INPUT[@]}"
        do
            python3 hanging_point_sim.py --hook $input
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

                "data/Hook1-hanging_exp/Hook1-hanging_exp_daily_5" \
                "data/Hook2-hanging_exp/Hook2-hanging_exp_daily_5" \
                "data/Hook11-hanging_exp/Hook11-hanging_exp_daily_5" \
                "data/Hook12-hanging_exp/Hook12-hanging_exp_daily_5" \
                "data/Hook15-hanging_exp/Hook15-hanging_exp_daily_5" \
                "data/Hook22-hanging_exp/Hook22-hanging_exp_daily_5" \
                "data/Hook23-hanging_exp/Hook23-hanging_exp_daily_5" \
                "data/Hook35-hanging_exp/Hook35-hanging_exp_daily_5" \
                "data/Hook40-hanging_exp/Hook40-hanging_exp_daily_5" \
                "data/Hook42-hanging_exp/Hook42-hanging_exp_daily_5" \
                "data/Hook44-hanging_exp/Hook44-hanging_exp_daily_5" \
                "data/Hook47-hanging_exp/Hook47-hanging_exp_daily_5" \
                "data/Hook48-hanging_exp/Hook48-hanging_exp_daily_5" \
                "data/Hook57-hanging_exp/Hook57-hanging_exp_daily_5" \
                "data/Hook67-hanging_exp/Hook67-hanging_exp_daily_5" \
                "data/Hook84-hanging_exp/Hook84-hanging_exp_daily_5" \
                "data/Hook122-hanging_exp/Hook122-hanging_exp_daily_5" \
                "data/Hook124-hanging_exp/Hook124-hanging_exp_daily_5" \
                "data/Hook136-hanging_exp/Hook136-hanging_exp_daily_5" \
                "data/Hook145-hanging_exp/Hook145-hanging_exp_daily_5" \
                "data/Hook186-hanging_exp/Hook186-hanging_exp_daily_5" \
                "data/Hook209-hanging_exp/Hook209-hanging_exp_daily_5" \
            )

        # the number of init poses
        if [ $# -eq 2 ]; then 
            CNT=$2
        else 
            CNT=100
        fi

        for input in "${INPUT[@]}"
        do
            python3 hanging_init_pose.py --input-json "$input.json" --max-cnt $CNT
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
            # 'keypoint_trajectory/hanging_exp_daily_11.json' \
            # 'keypoint_trajectory/hanging_exp_daily_23.json' \
            # 'keypoint_trajectory/hanging_exp_bag_70.json' \
            # 'keypoint_trajectory/hanging_exp_mug_67.json' \
            # 'keypoint_trajectory/hanging_exp_mug_115.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_14.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_19.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_39.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_48.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_62.json' \
            # 'keypoint_trajectory/hanging_exp_wrench_27.json' \
            # 'keypoint_trajectory/hanging_exp_wrench_35.json' 

            'keypoint_trajectory/hanging_exp_bag_5.json' \
            # 'keypoint_trajectory/hanging_exp_daily_5.json' \
            # 'keypoint_trajectory/hanging_exp_mug_59.json' \
            # 'keypoint_trajectory/hanging_exp_scissor_4.json' \
            # 'keypoint_trajectory/hanging_exp_wrench_1.json'

            # "keypoint_trajectory/hanging_exp_bag_5.json" \
            # "keypoint_trajectory/hanging_exp_bag_6.json" \
            # "keypoint_trajectory/hanging_exp_bag_70.json" \
            # "keypoint_trajectory/hanging_exp_daily_106.json" \
            # "keypoint_trajectory/hanging_exp_daily_114.json" \
            # "keypoint_trajectory/hanging_exp_daily_115.json" \
            # "keypoint_trajectory/hanging_exp_daily_118.json" \
            # "keypoint_trajectory/hanging_exp_daily_11.json" \
            # "keypoint_trajectory/hanging_exp_daily_23.json" \
            # "keypoint_trajectory/hanging_exp_daily_2.json" \
            # "keypoint_trajectory/hanging_exp_daily_41.json" \
            # "keypoint_trajectory/hanging_exp_daily_42.json" \
            # "keypoint_trajectory/hanging_exp_daily_5.json" \
            # "keypoint_trajectory/hanging_exp_daily_63.json" \
            # "keypoint_trajectory/hanging_exp_daily_71.json" \
            # "keypoint_trajectory/hanging_exp_daily_72.json" \
            # "keypoint_trajectory/hanging_exp_daily_7.json" \
            # "keypoint_trajectory/hanging_exp_daily_84.json" \
            # "keypoint_trajectory/hanging_exp_daily_85.json" \
            # "keypoint_trajectory/hanging_exp_daily_8.json" \
            # "keypoint_trajectory/hanging_exp_daily_97.json" \
            # "keypoint_trajectory/hanging_exp_mug_100.json" \
            # "keypoint_trajectory/hanging_exp_mug_112.json" \
            # "keypoint_trajectory/hanging_exp_mug_113.json" \
            # "keypoint_trajectory/hanging_exp_mug_115.json" \
            # "keypoint_trajectory/hanging_exp_mug_118.json" \
            # "keypoint_trajectory/hanging_exp_mug_11.json" \
            # "keypoint_trajectory/hanging_exp_mug_123.json" \
            # "keypoint_trajectory/hanging_exp_mug_126.json" \
            # "keypoint_trajectory/hanging_exp_mug_128.json" \
            # "keypoint_trajectory/hanging_exp_mug_129.json" \
            # "keypoint_trajectory/hanging_exp_mug_132.json" \
            # "keypoint_trajectory/hanging_exp_mug_135.json" \
            # "keypoint_trajectory/hanging_exp_mug_142.json" \
            # "keypoint_trajectory/hanging_exp_mug_145.json" \
            # "keypoint_trajectory/hanging_exp_mug_146.json" \
            # "keypoint_trajectory/hanging_exp_mug_147.json" \
            # "keypoint_trajectory/hanging_exp_mug_149.json" \
            # "keypoint_trajectory/hanging_exp_mug_150.json" \
            # "keypoint_trajectory/hanging_exp_mug_159.json" \
            # "keypoint_trajectory/hanging_exp_mug_166.json" \
            # "keypoint_trajectory/hanging_exp_mug_173.json" \
            # "keypoint_trajectory/hanging_exp_mug_181.json" \
            # "keypoint_trajectory/hanging_exp_mug_184.json" \
            # "keypoint_trajectory/hanging_exp_mug_193.json" \
            # "keypoint_trajectory/hanging_exp_mug_199.json" \
            # "keypoint_trajectory/hanging_exp_mug_19.json" \
            # "keypoint_trajectory/hanging_exp_mug_204.json" \
            # "keypoint_trajectory/hanging_exp_mug_205.json" \
            # "keypoint_trajectory/hanging_exp_mug_43.json" \
            # "keypoint_trajectory/hanging_exp_mug_59.json" \
            # "keypoint_trajectory/hanging_exp_mug_64.json" \
            # "keypoint_trajectory/hanging_exp_mug_67.json" \
            # "keypoint_trajectory/hanging_exp_mug_70.json" \
            # "keypoint_trajectory/hanging_exp_mug_73.json" \
            # "keypoint_trajectory/hanging_exp_mug_80.json" \
            # "keypoint_trajectory/hanging_exp_mug_82.json" \
            # "keypoint_trajectory/hanging_exp_mug_90.json" \
            # "keypoint_trajectory/hanging_exp_scissor_101.json" \
            # "keypoint_trajectory/hanging_exp_scissor_12.json" \
            # "keypoint_trajectory/hanging_exp_scissor_14.json" \
            # "keypoint_trajectory/hanging_exp_scissor_19.json" \
            # "keypoint_trajectory/hanging_exp_scissor_22.json" \
            # "keypoint_trajectory/hanging_exp_scissor_27.json" \
            # "keypoint_trajectory/hanging_exp_scissor_31.json" \
            # "keypoint_trajectory/hanging_exp_scissor_39.json" \
            # "keypoint_trajectory/hanging_exp_scissor_48.json" \
            # "keypoint_trajectory/hanging_exp_scissor_4.json" \
            # "keypoint_trajectory/hanging_exp_scissor_58.json" \
            # "keypoint_trajectory/hanging_exp_scissor_62.json" \
            # "keypoint_trajectory/hanging_exp_scissor_74.json" \
            # "keypoint_trajectory/hanging_exp_scissor_79.json" \
            # "keypoint_trajectory/hanging_exp_scissor_8.json" \
            # "keypoint_trajectory/hanging_exp_scissor_92.json" \
            # "keypoint_trajectory/hanging_exp_scissor_95.json" \
            # "keypoint_trajectory/hanging_exp_scissor_98.json" \
            # "keypoint_trajectory/hanging_exp_wrench_10.json" \
            # "keypoint_trajectory/hanging_exp_wrench_12.json" \
            # "keypoint_trajectory/hanging_exp_wrench_17.json" \
            # "keypoint_trajectory/hanging_exp_wrench_1.json" \
            # "keypoint_trajectory/hanging_exp_wrench_25.json" \
            # "keypoint_trajectory/hanging_exp_wrench_27.json" \
            # "keypoint_trajectory/hanging_exp_wrench_31.json" \
            # "keypoint_trajectory/hanging_exp_wrench_32.json" \
            # "keypoint_trajectory/hanging_exp_wrench_35.json" \
            # "keypoint_trajectory/hanging_exp_wrench_36.json" \
        )
        HOOK=($(ls keypoint_trajectory/Hook_90*) $(ls keypoint_trajectory/Hook_180*))

        rm "keypoint_trajectory/result.txt"
        for hook in "${HOOK[@]}"
        do
            for object in "${OBJECT[@]}"
            do
                echo "pair : $object $hook"
                python3 hanging_by_trajectory.py --obj $object --hook $hook
            done
        done
    
    elif [ $1 = 'showobj' ]; then 

        OBJ_DIRS=(
            'models/hook' \
            # 'models/geo_data/hanging_exp' \
            'models/geo_data/wrench' \
            'models/geo_data/scissor' \
            'models/geo_data/mug' \
            'models/geo_data/daily_object' \
        )
        for OBJ_DIR in "${OBJ_DIRS[@]}"
        do
            python3 show_model.py --input-dir $OBJ_DIR
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
