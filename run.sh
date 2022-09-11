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

    elif [ $1 = 'obj2urdf' ]; then 
        
        INPUT=('models/geo_data/hanging_exp')
        for input in "${INPUT[@]}"
        do
            python3 utils/obj2urdf.py --input-dir $input
        done

    elif [ $1 = 'vhacd' ]; then 
        
        INPUT=('models/geo_data/hanging_exp')
        for input in "${INPUT[@]}"
        do
            python3 vhacd.py --input-dir $input
        done
    
    elif [ $1 = 'hangenv' ]; then 
        
        max=20
        # INPUT='data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5'
        INPUT=( 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5' \
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
            rm "${input}_easy.csv"
            rm "${input}_medium.csv"
            rm "${input}_hard.csv"
            for i in `seq 1 $max`
            do
                python3 hanging_env.py --input-json "$input.json" --id $i > "$input.txt"
                python3 extract_log.py --input "$input.txt" --output "$input" 
                rm "$input.txt"
            done
        done

    elif [ $1 = 'hangenvhce' ]; then 
        
        max=20
        # INPUT='data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5'
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
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_bag_5' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_mug_59' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_scissor_4' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_bag_5' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_mug_59' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_scissor_4' \
                'data/Hook_60-hanging_exp/Hook_60-hanging_exp_wrench_1' \
                'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1' \
                'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_wrench_1' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_wrench_1' \
                'data/Hook_180-hanging_exp/Hook_180-hanging_exp_wrench_1'
            )
        for input in "${INPUT[@]}"
        do
            rm "${input}_hce_easy.csv"
            rm "${input}_hce_medium.csv"
            rm "${input}_hce_hard.csv"
            for i in `seq 1 $max`
            do
                python3 hanging_env_hce.py --input-json "$input.json" --id $i > "${input}_hce.txt"
                python3 extract_log.py --input "${input}_hce.txt" --output "${input}_hce" 
                rm "${input}_hce.txt"
            done
        done

    
    elif [ $1 = 'refine' ]; then 
        
        # INPUT='data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5'
        INPUT=( #'data/Hook_60-hanging_exp/Hook_60-hanging_exp_bag_5' \
                #'data/Hook_60-hanging_exp/Hook_60-hanging_exp_daily_5' \
                #'data/Hook_60-hanging_exp/Hook_60-hanging_exp_mug_59' \
                #'data/Hook_60-hanging_exp/Hook_60-hanging_exp_scissor_4' \
                #'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_bag_5' \
                #'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_daily_5' \
                #'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_mug_59' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_scissor_4' \
                #'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_bag_5' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_daily_5' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_mug_59' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_scissor_4' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_bag_5' \
                #'data/Hook_180-hanging_exp/Hook_180-hanging_exp_daily_5' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_mug_59' \
                #'data/Hook_180-hanging_exp/Hook_180-hanging_exp_scissor_4' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_bag_5' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_daily_5' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_mug_59' \
                # 'data/Hook_90-hanging_exp/Hook_90-hanging_exp_scissor_4' \
                # 'data/Hook_60-hanging_exp/Hook_60-hanging_exp_wrench_1' \
                # 'data/Hook_bar-hanging_exp/Hook_bar-hanging_exp_wrench_1' \
                # 'data/Hook_skew-hanging_exp/Hook_skew-hanging_exp_wrench_1' \
                'data/Hook_90-hanging_exp/Hook_90-hanging_exp_wrench_1' \
                # 'data/Hook_180-hanging_exp/Hook_180-hanging_exp_wrench_1'
            )
        for input in "${INPUT[@]}"
        do
            python3 hanging_pose_refine.py --input-json "$input.json"
        done

    elif [ $1 = 'hcetest' ]; then 

        OBJ='models/geo_data/hanging_exp'
        HOOK='models/hook'
        # THRESH=(0.0)
        THRESH=(0.1 0.05 0.0 -0.05 -0.1 -0.15 -0.2)
        for thresh in ${THRESH[@]}
        do
            python3 test_hce.py --obj $OBJ --hook $HOOK --thresh $thresh
        done
    else 
        echo "nope"
    fi

else 
    echo "should input at least one argument"
fi
