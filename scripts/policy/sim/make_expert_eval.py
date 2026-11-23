s = open("scripts/dukki_standup_eval.py").read()
s = s.replace("""        policy = runner.get_inference_policy(device=args.device)
""", """        policy = runner.get_inference_policy(device=args.device)
        if args.expert:
            # the BC teacher itself (alpha_stand.onnx), fed as in training: twist slot zeroed
            expert = runner.alg.expert

            def policy(obs, stochastic_output=False, _e=expert):
                obs = obs.clone()
                obs[_e.obs_groups[0]][:, 48:51] = 0.0
                return _e(obs)
""")
s = s.replace("""    args = ap.parse_args()""", """    ap.add_argument("--expert", action="store_true", help="run the distill task's BC expert instead of the checkpoint")
    args = ap.parse_args()""")
s = s.replace('print(f"== {args.checkpoint}', 'print(f"== {\'EXPERT \' if args.expert else \'\'}{args.checkpoint}')
open("../runs/standup_eval_expert.py", "w").write(s)
