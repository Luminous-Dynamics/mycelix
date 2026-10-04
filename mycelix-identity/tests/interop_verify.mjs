import fs from 'node:fs/promises';
import * as vc from '@digitalbazaar/vc';
import {DataIntegrityProof} from '@digitalbazaar/data-integrity';
import {createVerifyCryptosuite} from '@digitalbazaar/eddsa-jcs-2022-cryptosuite';
import {securityLoader} from '@digitalbazaar/security-document-loader';
import {contexts as credentialContexts} from '@digitalbazaar/credentials-context';
import dataIntegrityContext from '@digitalbazaar/data-integrity-context';
import multikeyContext from '@digitalbazaar/multikey-context';

const fixturePath = process.argv[2];
if(!fixturePath) throw new Error('usage: node interop_verify.mjs <fixture>');

const fixture = JSON.parse(await fs.readFile(fixturePath, 'utf8'));
if(fixture.cryptosuite !== 'eddsa-jcs-2022') throw new Error('unexpected cryptosuite');
if(fixture.schema_version !== 1) throw new Error('unexpected fixture schema');

const loader = securityLoader();
for(const [url, document] of credentialContexts) loader.addStatic(url, document);
loader.addStatic(dataIntegrityContext.CONTEXT_URL, dataIntegrityContext.CONTEXT);
loader.addStatic(multikeyContext.CONTEXT_URL, multikeyContext.CONTEXT);

for(const doc of [fixture.issuerDidDocument, fixture.holderDidDocument]) {
  loader.addStatic(doc.id, doc);
  for(const method of doc.verificationMethod ?? []) {
    loader.addStatic(method.id, {
      '@context': multikeyContext.CONTEXT_URL,
      id: method.id,
      type: method.type,
      controller: method.controller,
      publicKeyMultibase: method.publicKeyMultibase
    });
  }
}

const documentLoader = loader.build();
const suite = new DataIntegrityProof({
  cryptosuite: createVerifyCryptosuite()
});

const credentialResult = await vc.verifyCredential({
  credential: fixture.credential,
  suite,
  documentLoader
});
if(!credentialResult.valid) {
  throw new Error('Digital Bazaar rejected Mycelix credential: ' +
    JSON.stringify(credentialResult));
}

const presentationResult = await vc.verify({
  presentation: fixture.presentation,
  challenge: fixture.presentationChallenge,
  domain: fixture.presentationDomain,
  suite,
  documentLoader
});
if(!presentationResult.valid) {
  throw new Error('Digital Bazaar rejected Mycelix presentation: ' +
    JSON.stringify(presentationResult));
}

const credentialSubject = fixture.credential?.credentialSubject;
if(!credentialSubject || typeof credentialSubject !== 'object' ||
   !Object.prototype.hasOwnProperty.call(credentialSubject, 'degree')) {
  throw new Error('fixture credentialSubject.degree is required for the tamper test');
}

const vpProof = fixture.presentation?.proof;
if(!vpProof || vpProof.challenge !== fixture.presentationChallenge ||
   vpProof.domain !== fixture.presentationDomain) {
  throw new Error('fixture challenge/domain mismatch');
}

async function mustReject(label, fn) {
  const result = await fn();
  if(result?.valid === false) return;
  throw new Error(label + ' unexpectedly verified or did not return valid=false');
}

const tamperedCredential = structuredClone(fixture.credential);
tamperedCredential.credentialSubject.degree = 'Tampered claim';
await mustReject('tampered credential', () => vc.verifyCredential({
  credential: tamperedCredential,
  suite,
  documentLoader
}));

await mustReject('wrong presentation challenge', () => vc.verify({
  presentation: fixture.presentation,
  challenge: fixture.presentationChallenge + '-wrong',
  domain: fixture.presentationDomain,
  suite,
  documentLoader
}));

await mustReject('wrong presentation domain', () => vc.verify({
  presentation: fixture.presentation,
  challenge: fixture.presentationChallenge,
  domain: fixture.presentationDomain + '-wrong',
  suite,
  documentLoader
}));

const tamperedPresentation = structuredClone(fixture.presentation);
if(typeof tamperedPresentation.proof?.proofValue !== 'string' ||
   tamperedPresentation.proof.proofValue.length < 2) {
  throw new Error('fixture presentation proofValue is required for the tamper test');
}
tamperedPresentation.proof.proofValue =
  tamperedPresentation.proof.proofValue.slice(0, -1) +
  (tamperedPresentation.proof.proofValue.endsWith('A') ? 'B' : 'A');

await mustReject('tampered presentation proof', () => vc.verify({
  presentation: tamperedPresentation,
  challenge: fixture.presentationChallenge,
  domain: fixture.presentationDomain,
  suite,
  documentLoader
}));

const credentialVerificationMethodId = fixture.credential?.proof?.verificationMethod;
if(typeof credentialVerificationMethodId !== 'string' || !credentialVerificationMethodId) {
  throw new Error('fixture credential proof.verificationMethod is required');
}
const issuerKeyMethod = fixture.issuerDidDocument?.verificationMethod?.find(
  method => method.id === credentialVerificationMethodId
);
if(!issuerKeyMethod?.publicKeyMultibase) {
  throw new Error(
    'fixture issuer verification method for credential proof.verificationMethod is required'
  );
}
const issuerKeyIndex = fixture.issuerDidDocument.verificationMethod.findIndex(
  method => method.id === credentialVerificationMethodId
);
if(issuerKeyIndex < 0) {
  throw new Error('credential proof verification method is absent from issuer DID document');
}
const tamperedIssuerDocument = structuredClone(fixture.issuerDidDocument);
tamperedIssuerDocument.verificationMethod[issuerKeyIndex].publicKeyMultibase =
  tamperedIssuerDocument.verificationMethod[issuerKeyIndex].publicKeyMultibase.slice(0, -1) +
  (tamperedIssuerDocument.verificationMethod[issuerKeyIndex].publicKeyMultibase.endsWith('A') ? 'B' : 'A');

const tamperedLoader = securityLoader();
for(const [url, document] of credentialContexts) tamperedLoader.addStatic(url, document);
tamperedLoader.addStatic(dataIntegrityContext.CONTEXT_URL, dataIntegrityContext.CONTEXT);
tamperedLoader.addStatic(multikeyContext.CONTEXT_URL, multikeyContext.CONTEXT);
tamperedLoader.addStatic(tamperedIssuerDocument.id, tamperedIssuerDocument);
for(const method of tamperedIssuerDocument.verificationMethod ?? []) {
  tamperedLoader.addStatic(method.id, {
    '@context': multikeyContext.CONTEXT_URL,
    id: method.id,
    type: method.type,
    controller: method.controller,
    publicKeyMultibase: method.publicKeyMultibase
  });
}
for(const method of fixture.holderDidDocument.verificationMethod ?? []) {
  tamperedLoader.addStatic(method.id, {
    '@context': multikeyContext.CONTEXT_URL,
    id: method.id,
    type: method.type,
    controller: method.controller,
    publicKeyMultibase: method.publicKeyMultibase
  });
}
tamperedLoader.addStatic(fixture.holderDidDocument.id, fixture.holderDidDocument);

await mustReject('tampered issuer verification key', () => vc.verifyCredential({
  credential: fixture.credential,
  suite,
  documentLoader: tamperedLoader.build()
}));

console.log(JSON.stringify({
  independent_implementation: 'Digital Bazaar',
  cryptosuite: fixture.cryptosuite,
  credential_valid: credentialResult.valid,
  presentation_valid: presentationResult.valid,
  challenge_validated: true,
  domain_present: true,
  tampered_credential_rejected: true,
  wrong_challenge_rejected: true,
  wrong_domain_rejected: true,
  tampered_presentation_rejected: true,
  tampered_issuer_key_rejected: true
}, null, 2));
