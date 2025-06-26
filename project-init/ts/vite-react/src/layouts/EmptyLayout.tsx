import React from 'react';

const EmptyLayout = ({ children }: { children: React.ReactNode }) => {
    return (
        <>
            <main>{children}</main>
        </>
    );
};

export default EmptyLayout;
