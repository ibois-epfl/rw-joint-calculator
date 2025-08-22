The python package is currently too limited to be published in pypi, so we stick to local installation. The compomnents are also too much in development for a yak to be published, so only precompiled
## Install the library

To install the library, open a terminal, create and activate the conda environment:
```bash
conda env create -f ./environment.yml
conda activate rw-joint-calculator
```

Then you can run the automatized script:
```bash
invoke install-lib
```

## Drag-and-drop the components

In the [latest release](https://github.com/ibois-epfl/rw-joint-calculator/releases), download the `.ghuser` files and drag-and-drop them in grasshopper. That is it, you can use this plug-in ! :)
