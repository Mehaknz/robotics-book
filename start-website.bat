@echo off
echo =================================================================
echo  Starting the Humanoid Robotics Book Website...
echo =================================================================
echo.
echo Navigating to the website directory...
cd humanoid-robotics-book/frontend
echo.
echo Installing dependencies (this may take a few minutes)...
npm install
echo.
echo Starting the development server...
echo Your website will be available at http://localhost:3000
echo.
npm start
