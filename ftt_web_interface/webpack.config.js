/* eslint-disable no-undef */
const path = require("path");
const HtmlWebpackPlugin = require('html-webpack-plugin');

module.exports = {
  mode: "development",
  entry: {
    index: "./src/index.js",
    config: "./src/config.js",
  },
  output: {
    filename: "[name].js",
    path: path.resolve(__dirname, "assets", "scripts"),
    publicPath: "assets/scripts/",
  },
  devServer: {
    contentBase: "./",
  },
  devtool: "cheap-module-eval-source-map",
  plugins: [
    new HtmlWebpackPlugin({
      template: './index.html',
      inject: false,
      chunks: ['index'],
      filename: 'index.html'
    }),
    new HtmlWebpackPlugin({
      template: './config.html',
      inject: false,
      chunks: ['config'],
      filename: 'config.html'
    })
  ]
};
