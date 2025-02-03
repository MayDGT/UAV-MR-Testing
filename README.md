## Getting Started

* Following [this guide](https://github.com/skhatiri/Aerialist#using-hosts-cli) to install Aerialist project
       
* `cd Aerialist/samples` and clone this project

* Create an virtual enviornment and activate it:
     * `conda create -n UAV-MR-Testing python=3.9`
     * `conda activate UAV-MR-Testing`

* Create the required directories and packages:
     * `cd UAV-MR-Testing/snippets`
     * `sudo mkdir -p logs results/logs`
     * `pip install -r requirements.txt`
 
 * (Optional) Set your own configurations in `config.yaml`
 
* Run the experiment:
     * `python rvp_search.py`
