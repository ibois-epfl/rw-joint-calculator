from invoke import task
import os

from src.joint_calc.__init__ import __version__ as VERSION

PATH_COMPONENTS = "./src/components"
PATH_COMPONENTIZER = "./componentizer/ghcomponentizer.py"
PATH_DLL_COMPONENTIZER = "./componentizer/ghio"
PATH_OUT_COMPONENTS = "./src/build/latest_compiled_components"
PREFIX = "RWJC"


@task
def ghcomponentizer(ctx):
    ctx.run(
        "python {} {} {} --ghio {} --version {} --prefix {}".format(
            PATH_COMPONENTIZER,
            PATH_COMPONENTS,
            PATH_OUT_COMPONENTS,
            PATH_DLL_COMPONENTIZER,
            VERSION,
            PREFIX,
        )
    )


@task
def install_lib(ctx):
    user = os.getlogin()
    python_path = f"C:\\Users\\{user}\\.rhinocode\\py39-rh8\\python.exe"
    ctx.run(f'"{python_path}" -m pip install ./src/python_library')
