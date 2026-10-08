import { theme, type ThemeConfig } from "antd";

// The one theme for the app. styles.css reads it as --ant-* variables under the `app-theme` class;
// canvas and WebGL drawing read the same values from `tokens`.
export const antdTheme: ThemeConfig = {
  algorithm: theme.darkAlgorithm,
  cssVar: { key: "app-theme" },
};

export const tokens = theme.getDesignToken(antdTheme);
