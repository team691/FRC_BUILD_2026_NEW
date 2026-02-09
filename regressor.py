import statsmodels
import numpy as np
import statsmodels.api as stats
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures

df = pd.read_json("data.json")

X = df[["dist_ft", "accuracy"]]
y = df["speed"]

X = stats.add_constant(X)

poly_features = PolynomialFeatures(degree=2)
xp = poly_features.fit_transform(X)
model = stats.OLS(y, X).fit()

# summary read
try:
    new_df = pd.read_html(model.summary().tables[1].as_html(), header=0, index_col=0)[0]

    a = df["coef"].values[1]
    print(a)
except Exception as e:
    print(f"failed:\n{e}")

print(f"shape: {xp.shape}")
with open('params.tx', "w") as file:
    file.write(str(model.params))