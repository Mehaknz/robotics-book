import React from 'react';
import Layout from '@theme/Layout';
import HumanoidRoboticsComponent from '../components/index'; // Adjust path if necessary
   
export default function MyCustomContent() {
  return (
    <Layout title="My Custom Content">
      <main>
        <HumanoidRoboticsComponent />
      </main>
    </Layout>
  );
}