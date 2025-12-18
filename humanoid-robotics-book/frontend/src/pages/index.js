import React from 'react';
import clsx from 'clsx';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import styles from './index.module.css';
import AuthModals from '../components/AuthModals'; // Import AuthModals component

import Hero from '../components/Hero';

import SyllabusGrid from '../components/SyllabusGrid'; // Import the new component
import CustomFooter from '../components/CustomFooter'; // Import the new CustomFooter component

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <Hero />
      <main>

        <SyllabusGrid /> {/* Render the new component */}
      </main>
      <CustomFooter /> {/* Render the new CustomFooter component */}
      <AuthModals /> {/* Render AuthModals component */}
    </Layout>
  );
}
