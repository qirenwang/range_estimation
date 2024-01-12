import warnings
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import RandomForestRegressor, AdaBoostRegressor
from sklearn.model_selection import train_test_split, ShuffleSplit, cross_validate
from sklearn.preprocessing import LabelEncoder, OneHotEncoder, StandardScaler
from sklearn.metrics import mean_squared_error, r2_score, mean_absolute_error
import keras
import tensorflow as tf
from keras.layers import Dense

warnings.filterwarnings(action="ignore")
pd.set_option('display.width', 200)
pd.set_option('display.max_columns', 20)

old_path = "/home/qiren/ros2_ws/src/range_estimation/range_estimation/data.csv"
new_path = ("/home/qiren/ros2_ws/src/range_estimation/range_estimation/new_data.csv")



# remove missing values
ds = pd.read_csv(filepath_or_buffer=old_path)
ds = ds[pd.notnull(obj=ds['quantity(kWh)'])]
ds = ds[pd.notnull(obj=ds['avg_speed(km/h)'])]
ds.to_csv(path_or_buf=new_path)

# load the data
dataset = pd.read_csv(filepath_or_buffer=new_path)
X = dataset.iloc[:, 5:14].values
y = dataset.iloc[:, 14].values

# preprocessing
label_encoder_1 = LabelEncoder()
X[:, 1] = label_encoder_1.fit_transform(y=X[:, 1])
label_encoder_2 = LabelEncoder()
X[:, 5] = label_encoder_2.fit_transform(y=X[:, 5])

onehot_encoder = OneHotEncoder() # categorical_features=[5]
X = onehot_encoder.fit_transform(X=X).toarray()
X = X[:, 1:]

X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, shuffle=True)

sc = StandardScaler()
X_train = sc.fit_transform(X=X_train)
X_test = sc.fit_transform(X=X_test)

def do_kfold(model):
    cv = ShuffleSplit(n_splits=10, test_size=0.5, random_state=2)
    results = cross_validate(estimator=model, X=X, y=y, cv=cv, scoring=['neg_mean_absolute_error', 'r2'], n_jobs=-1)
    mae_values = results['test_neg_mean_absolute_error']
    r2_scores = results['test_r2']
    return mae_values, r2_scores


def do_fit_predict(model):
    model.fit(X_train, y_train)
    training_pred = model.predict(X_train)
    test_pred = model.predict(X_test)
    return training_pred, test_pred


def report_cross_val_results(mae_values, r2_scores):
    print("average MAE values (bias) is:", abs(round(number=mae_values.mean(), ndigits=3)))
    print("std deviation of MAE values (variance) is:", round(number=mae_values.std(), ndigits=3))
    best_mae = sorted(mae_values, reverse=False)[-1]
    print("best MAE value is:", abs(round(number=best_mae, ndigits=3)))

    print("average r2 scores (bias) is:", round(number=r2_scores.mean() * 100, ndigits=3))
    print("std deviation of r2 scores (variance) is:", round(number=r2_scores.std() * 100, ndigits=3))
    best_r2 = sorted(r2_scores, reverse=False)[-1] * 100
    print("best r2 score is:", round(number=best_r2, ndigits=3))
    print("-------------------------------")


def report_results(training_pred, test_pred):
    print("RMSE on training data: %.3f" % np.sqrt(mean_squared_error(y_true=y_train, y_pred=training_pred)))
    print("RMSE on test data: %.3f" % np.sqrt(mean_squared_error(y_true=y_test, y_pred=test_pred)))
    print("MAE on training data: %.3f" % mean_absolute_error(y_true=y_train, y_pred=training_pred))
    print("MAE on test data: %.3f" % mean_absolute_error(y_true=y_test, y_pred=test_pred))
    print("variance score on training data: %.3f" % r2_score(y_true=y_train, y_pred=training_pred))
    print("variance score on test data: %.3f" % r2_score(y_true=y_test, y_pred=test_pred))
    print("-------------------------------")


def main():
    # linear regression model
    linear_regressor = LinearRegression()

    # KFold cross-validation
    print("\n ------ Linear Regression CrossVal ------")
    reg_mae_values, reg_r2_scores = do_kfold(model=linear_regressor)
    report_cross_val_results(mae_values=reg_mae_values, r2_scores=reg_r2_scores)

    # train linear regression model
    print("\n ------ Linear Regression TrainTest ------")
    reg_training_pred, reg_test_pred = do_fit_predict(model=linear_regressor)
    report_results(training_pred=reg_training_pred, test_pred=reg_test_pred)
    #
    # # MLP model
    # mlp = MLPRegressor(hidden_layer_sizes=(10,), max_iter=1000, n_iter_no_change=100, activation='relu', solver='adam', verbose=False, warm_start=False)
    #
    # # KFold cross-validation
    # print("\n ------ MLP CrossVal ------")
    # mlp_mae_values, mlp_r2_scores = do_kfold(model=mlp)
    # report_cross_val_results(mae_values=mlp_mae_values, r2_scores=mlp_r2_scores)
    #
    # # train MLP model
    # print("\n ------ MLP TrainTest ------")
    # mlp_training_pred, mlp_test_pred = do_fit_predict(model=mlp)
    # report_results(training_pred=mlp_training_pred, test_pred=mlp_test_pred)

    # # random forest model
    # rf = RandomForestRegressor(n_estimators=200, criterion="mae", warm_start=False)
    #
    # # # KFold cross-validation
    # # print("\n ------ Random Forest CrossVal ------")
    # # rf_mae_values, rf_r2_scores = do_kfold(model=rf)
    # # report_cross_val_results(mae_values=rf_mae_values, r2_scores=rf_r2_scores)
    #
    # # train random forest model
    # print("\n ------ Random Forest TrainTest ------")
    # rf_train_pred, rf_test_pred = do_fit_predict(model=rf)
    # report_results(training_pred=rf_train_pred, test_pred=rf_test_pred)

    # AdaBoost model
    ab = AdaBoostRegressor(n_estimators=50, learning_rate=1.)

    # KFold cross-validation
    print("\n ------ AdaBoost CrossVal ------")
    ab_mae_values, ab_r2_scores = do_kfold(model=ab)
    report_cross_val_results(mae_values=ab_mae_values, r2_scores=ab_r2_scores)

    # train AdaBoost model
    print("\n ------ AdaBoost TrainTest ------")
    ab_train_pred, ab_test_pred = do_fit_predict(model=ab)
    report_results(training_pred=ab_train_pred, test_pred=ab_test_pred)

    # # deep multi-layer perceptron model
    # def build_regressor():
    #     regressor = tf.keras.models.Sequential()
    #     regressor.add(Dense(units=100, kernel_initializer='uniform', activation='relu', input_dim=len(X[0])))
    #     regressor.add(Dense(units=50, kernel_initializer='uniform', activation='relu'))
    #     regressor.add(Dense(units=25, kernel_initializer='uniform', activation='relu'))
    #     regressor.add(Dense(units=13, kernel_initializer='uniform', activation='relu'))
    #     regressor.add(Dense(units=7, kernel_initializer='uniform', activation='relu'))
    #     regressor.add(Dense(units=1, kernel_initializer='uniform', activation='linear'))
    #     regressor.compile(optimizer='adam', loss='mean_absolute_error')
    #     return regressor
    #
    # deep_mlp = KerasRegressor(build_fn=build_regressor, batch_size=16, epochs=10, verbose=False)

    # KFold cross-validation
    # print("\n ------ Deep MLP CrossVal ------")
    # deep_mlp_mae_values, deep_mlp_r2_scores = do_kfold(model=deep_mlp)
    # report_cross_val_results(mae_values=deep_mlp_mae_values, r2_scores=deep_mlp_r2_scores)
    #
    # # train deep mlp model
    # print("\n ------ Deep MLP TrainTest ------")
    # deep_mlp_train_pred, deep_mlp_test_pred = do_fit_predict(model=deep_mlp)
    # report_results(training_pred=deep_mlp_train_pred, test_pred=deep_mlp_test_pred)



    """plot driving range based on the battery quantity"""
    quantity = X[:, 2]
    distance = y
    quantity = np.reshape(quantity, newshape=(-1, 1))
    distance = np.reshape(distance, newshape=(-1, 1))

    quantity_linear_reg = LinearRegression()
    quantity_linear_reg.fit(X=quantity, y=distance)
    q_slope = quantity_linear_reg.coef_[0]
    q_intercept = quantity_linear_reg.intercept_
    q_predicted_distances = q_intercept + q_slope * quantity

    fig = plt.figure()
    plt.scatter(x=quantity, y=distance, s=15, c='black', linewidths=0.1)
    plt.plot(quantity, q_predicted_distances, c='red', linewidth=2)
    plt.legend(('fitted line', 'data records'), loc='lower right')
    plt.title(label='Linear Regression Plot')
    plt.xlabel(xlabel='quantity (kWh)'), plt.ylabel(ylabel='driving range (km)')
    plt.show()
    # fig.savefig('range_to_quantity.png')


    """plot driving range based on the average speed"""
    avg_speed = X[:, 9]
    avg_speed = np.reshape(avg_speed, newshape=(-1, 1))

    speed_linear_reg = LinearRegression()
    speed_linear_reg.fit(X=avg_speed, y=distance)
    s_slope = speed_linear_reg.coef_[0]
    s_intercept = speed_linear_reg.intercept_
    s_predicted_distances = s_intercept + s_slope * quantity

    fig = plt.figure()
    plt.scatter(x=avg_speed, y=distance, s=15, c='orange', linewidths=0.1)
    plt.plot(quantity, s_predicted_distances, c='blue', linewidth=2)
    plt.legend(('fitted line', 'data records'), loc='upper left')
    plt.title(label='Linear Regression Plot')
    plt.xlabel(xlabel='average speed (km/h)'), plt.ylabel(ylabel='driving range (km)')
    plt.xlim(-5, 110), plt.ylim(-30, 650)
    plt.show()
    #fig.savefig('range_to_speed.png')


if __name__ == "__main__":
    main()