    if (control_mode == full_auto_control)
    {
        static int32_t  ba_base = 0, aa_base = 0;
        static float    ba_int = 0.0f, ba_lerr = 0.0f;
        static float    aa_int = 0.0f, aa_lerr = 0.0f;
        static uint8_t  ba_init = 0U, aa_init = 0U;
        static Flex_TargetPos ba_prev = flex_pos0, aa_prev = flex_pos0;
        float bcmd = 0.0f;
        float acmd = 0.0f;
        if (kfs_below_cmd != kfs_below_cmd_stop)
        {
            Flex_TargetPos t = (Flex_TargetPos)((int)kfs_below_cmd - 1);
            if (t != ba_prev || ba_init == 0U)
            {
                ba_base = kfs_below.round_cnt;
                ba_int  = 0.0f;
                ba_lerr = 0.0f;
                ba_init = 1U;
                ba_prev = t;
            }
            {
                float tr  = (float)ba_base + kfs_below_pos_param.pos_rounds[t];
                float err = tr - (float)kfs_below.round_cnt;
                float der;
                ba_int += err;
                if (ba_int > kfs_below_pos_param.pos_i_limit)
                    ba_int = kfs_below_pos_param.pos_i_limit;
                if (ba_int < -kfs_below_pos_param.pos_i_limit)
                    ba_int = -kfs_below_pos_param.pos_i_limit;
                der  = err - ba_lerr;
                ba_lerr = err;
                bcmd = (kfs_below_pos_param.pos_kp * err
                     + kfs_below_pos_param.pos_ki * ba_int
                     + kfs_below_pos_param.pos_kd * der) * 200.0f;
            }
        }
        if (kfs_above_cmd != kfs_above_cmd_stop)
        {
            Flex_TargetPos t = (Flex_TargetPos)((int)kfs_above_cmd - 1);
            if (t != aa_prev || aa_init == 0U)
            {
                aa_base = kfs_above.round_cnt;
                aa_int  = 0.0f;
                aa_lerr = 0.0f;
                aa_init = 1U;
                aa_prev = t;
            }
            {
                float tr  = (float)aa_base - kfs_above_pos_param.pos_rounds[t];
                float err = tr - (float)kfs_above.round_cnt;
                float der;
                aa_int += err;
                if (aa_int > kfs_above_pos_param.pos_i_limit)
                    aa_int = kfs_above_pos_param.pos_i_limit;
                if (aa_int < -kfs_above_pos_param.pos_i_limit)
                    aa_int = -kfs_above_pos_param.pos_i_limit;
                der  = err - aa_lerr;
                aa_lerr = err;
                acmd = (kfs_above_pos_param.pos_kp * err
                     + kfs_above_pos_param.pos_ki * aa_int
                     + kfs_above_pos_param.pos_kd * der) * 200.0f;
            }
        }
        kfs_above.PID_Calculate(&kfs_above, acmd);
        kfs_below.PID_Calculate(&kfs_below, bcmd);
    }
    else