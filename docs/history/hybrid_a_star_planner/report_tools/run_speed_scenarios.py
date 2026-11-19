from pathlib import Path
import shlex,subprocess,sys
workspace=Path(__file__).resolve().parents[3]
base=workspace/'build/hybrid_a_star_planner'
flags=(base/'CMakeFiles/test_v4.dir/flags.make').read_text()
includes=next(l.split(' = ',1)[1] for l in flags.splitlines() if l.startswith('CXX_INCLUDES'))
link=shlex.split((base/'CMakeFiles/test_v4.dir/link.txt').read_text())
for kind in sys.argv[1:] or ['before','after']:
 obj='/tmp/speed_'+kind+'.o';binary='/tmp/scenarios_'+kind
 cmd=['c++','-std=c++17','-O2']+shlex.split(includes)+(['-DUSV_BASELINE','-DUSV_BASELINE_SOURCE="'+str(workspace/'artifacts/hybrid_speed_update/planner_before.cpp')+'"'] if kind=='before' else [])+['-c',str(workspace/'src/hybrid_a_star_planner/test/test_speed_scenarios.cpp'),'-o',obj]
 subprocess.run(cmd,cwd=base,check=True)
 cmd=link[:];cmd[cmd.index('CMakeFiles/test_v4.dir/test/test_v4.cpp.o')]=obj;cmd[cmd.index('-o')+1]=binary
 cmd=[v for v in cmd if v not in ['gtest/libgtest_main.a','gtest/libgtest.a']]
 subprocess.run(cmd,cwd=base,check=True)
 subprocess.run([binary],cwd=workspace,env=__import__('os').environ|{'USV_REPORT_OUTPUT':f'artifacts/hybrid_speed_update/{kind}'},check=True)
