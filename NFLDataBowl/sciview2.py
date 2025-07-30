import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Example: Data Visualization

games = '~/NFL/games.csv'

myGames = pd.read_csv(games)
print(myGames)